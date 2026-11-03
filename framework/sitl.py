"""PX4 SITL container lifecycle, MAVSDK server, connection, readiness and parameters."""

from __future__ import annotations

import logging
import math
import socket
import subprocess
from pathlib import Path

import mavsdk_grpc
from mavsdk_grpc import System

from framework.assertions import FlightAssertionError, call, wait_for_stream
from framework.config import REPO_ROOT, Config

SCRIPTS = REPO_ROOT / "scripts"
ARTIFACTS = REPO_ROOT / "artifacts"
# Spike S3: PX4 keeps these values as configured, so the framework scales them.
LOSS_TIMEOUT_PARAMS = ("COM_DL_LOSS_T", "COM_RC_LOSS_T")

log = logging.getLogger(__name__)


class SitlInstance:
    """One PX4 SITL container with a fresh data directory."""

    def __init__(self, cfg: Config, test_id: str, params: dict):
        self.cfg = cfg
        self.params = dict(params)
        self.name = f"sim-{test_id}"
        self.data_dir = ARTIFACTS / "ulog" / test_id
        self.log_file = ARTIFACTS / "logs" / f"{test_id}.log"

    def start(self) -> None:
        # A leftover directory would carry parameters and missions into this test.
        self.data_dir.mkdir(parents=True, exist_ok=False)
        px4 = self.cfg.px4
        timeout_s = int(self.cfg.timeouts_wall_s.container_start)
        cmd = [
            str(SCRIPTS / "start_sitl.sh"),
            "--image", px4.local_tag,
            "--name", self.name,
            "--log-dir", str(self.data_dir),
            "--model", px4.model,
            "--speed-factor", str(px4.speed_factor),
            "--home-lat", str(px4.home.lat),
            "--home-lon", str(px4.home.lon),
            "--home-alt", str(px4.home.alt),
            "--timeout", str(timeout_s),
        ]
        for name, value in self.params.items():
            cmd += ["--param", f"{name}={value}"]
        result = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=timeout_s + 60
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"start_sitl.sh exited {result.returncode}:\n{result.stdout}{result.stderr}"
            )

    def stop(self) -> None:
        cmd = [str(SCRIPTS / "stop_sitl.sh"), "--name", self.name, "--log-file", str(self.log_file)]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)
        except Exception:  # teardown must not hide the test's own failure
            log.exception("stop_sitl.sh failed for %s", self.name)


class MavsdkServer:
    """The bundled mavsdk_server, started and stopped once per test."""

    def __init__(self, cfg: Config, log_path: Path):
        self.cfg = cfg
        self.log_path = log_path
        self.port: int | None = None
        self._process: subprocess.Popen | None = None

    def start(self) -> None:
        udp_port = self.cfg.mavlink.api_port
        _require_free_udp_port(udp_port)
        self.port = _free_tcp_port()
        binary = Path(mavsdk_grpc.__file__).parent / "bin" / "mavsdk_server"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [str(binary), "-p", str(self.port), "--sysid", "245", "--compid", "190",
               self.cfg.mavlink.api_url]
        # The child keeps its own copy of the file descriptor.
        with open(self.log_path, "w") as log_file:
            self._process = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT)

    def stop(self) -> None:
        if self._process is None or self._process.poll() is not None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait()

    def status(self) -> str:
        if self._process is None:
            return "not started"
        code = self._process.poll()
        state = "running" if code is None else f"exited with {code}"
        try:
            tail = self.log_path.read_text().splitlines()[-10:]
        except OSError:
            tail = []
        return f"mavsdk_server {state}; last output:\n" + "\n".join(tail)


def _require_free_udp_port(port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(("0.0.0.0", port))
        except OSError as exc:
            raise RuntimeError(f"UDP port {port} is in use (another mavsdk_server?)") from exc


def _free_tcp_port() -> int:
    # Bind to port 0 and reuse the number; one test per runner makes the race harmless.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


async def connect(cfg: Config, server: MavsdkServer) -> System:
    drone = System(mavsdk_server_address="localhost", port=server.port)
    timeout_s = cfg.timeouts_wall_s.connect
    try:
        await call(drone.connect(), cfg, "connect to mavsdk_server")
        await wait_for_stream(drone.core.connection_state(), lambda s: s.is_connected,
                              timeout_s, "PX4 connected")
    except FlightAssertionError as exc:
        raise FlightAssertionError("PX4 connected", f"{exc}\n{server.status()}") from None
    return drone


async def wait_ready(drone: System, cfg: Config) -> None:
    await wait_for_stream(
        drone.telemetry.health(),
        lambda h: h.is_global_position_ok and h.is_home_position_ok,
        cfg.timeouts_wall_s.health,
        "global position and home position OK",
    )


async def wait_armable(drone: System, cfg: Config) -> None:
    await wait_for_stream(drone.telemetry.health(), lambda h: h.is_armable,
                          cfg.timeouts_wall_s.armable, "vehicle armable")


def scaled_loss_params(params: dict, cfg: Config) -> dict:
    """Scale loss timeouts by the speed factor (spike S3 gave outcome (a) for both)."""
    scaled = dict(params)
    factor = cfg.px4.speed_factor
    if "COM_DL_LOSS_T" in scaled:  # INT32 in PX4
        scaled["COM_DL_LOSS_T"] = round(scaled["COM_DL_LOSS_T"] * factor)
    if "COM_RC_LOSS_T" in scaled:  # FLOAT in PX4
        scaled["COM_RC_LOSS_T"] = float(scaled["COM_RC_LOSS_T"]) * factor
    return scaled


async def assert_params(drone: System, boot_params: dict, cfg: Config) -> None:
    """Fail unless PX4 holds every boot parameter, using the type PX4 reports."""
    all_params = await call(drone.param.get_all_params(), cfg, "read all PX4 parameters")
    ints = {p.name: p.value for p in all_params.int_params}
    floats = {p.name: p.value for p in all_params.float_params}
    mismatches = []
    for name, expected in boot_params.items():
        if name in ints:
            actual, ok = ints[name], ints[name] == int(expected)
        elif name in floats:
            actual = floats[name]
            ok = math.isclose(actual, float(expected), rel_tol=1e-6, abs_tol=1e-9)
        else:
            actual, ok = "unknown to PX4", False
        if not ok:
            mismatches.append(f"  {name}: expected {expected}, actual {actual}")
    if mismatches:
        raise FlightAssertionError("PX4 holds every boot parameter", "\n" + "\n".join(mismatches))
