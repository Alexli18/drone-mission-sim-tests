"""ROS container lifecycle: start a launch file, stop it, read its log.

Imports only the standard library, PyYAML and ``framework``.
"""

from __future__ import annotations

import asyncio
import logging
import subprocess
from pathlib import Path

import yaml

from framework.assertions import FlightAssertionError, wait_until
from framework.config import REPO_ROOT, Config
from framework.sitl import ARTIFACTS, SCRIPTS

LOG_POLL_S = 0.5

log = logging.getLogger(__name__)


class Ros2Instance:
    """One ROS container running ``ros2 launch`` with a fresh output directory.

    ``launch`` goes to ``ros2 launch`` verbatim: ``"PKG FILE"`` or a path under
    ``/work`` (the repository, mounted read-only). ``out_dir`` is mounted at
    ``/out`` and ``in_dir`` (replay) at ``/in``.
    """

    def __init__(self, cfg: Config, test_id: str, *, name_suffix: str, launch: str,
                 out_dir: Path, params: dict, launch_args: dict | None = None,
                 in_dir: Path | None = None, image: str | None = None):
        self.cfg = cfg
        self.name = f"ros-{test_id}-{name_suffix}"
        self.launch = launch
        self.out_dir = Path(out_dir)
        self.params = params
        self.launch_args = dict(launch_args or {})
        self.in_dir = None if in_dir is None else Path(in_dir)
        self.image = image or cfg.ros.image_tag
        self.log_file = ARTIFACTS / "logs" / f"{test_id}.ros-{name_suffix}.log"

    def start(self) -> None:
        # A leftover directory would mix this run's bag and decisions with an older one.
        self.out_dir.mkdir(parents=True, exist_ok=False)
        with open(self.out_dir / "params.yaml", "w") as f:
            yaml.safe_dump(self.params, f, sort_keys=False)
        ros = self.cfg.ros
        timeout_s = int(self.cfg.timeouts_wall_s.ros_start)
        # Ports always come from config.yaml, never from the launch defaults.
        args = {
            "params": "/out/params.yaml",
            "out": "/out",
            "agent_port": ros.dds_agent_port,
            "rosbridge_address": ros.rosbridge_address,
            "rosbridge_port": ros.rosbridge_port,
            **self.launch_args,
        }
        cmd = [
            str(SCRIPTS / "start_ros2.sh"),
            "--image", self.image,
            "--name", self.name,
            "--repo-dir", str(REPO_ROOT),
            "--out-dir", str(self.out_dir),
            "--launch", self.launch,
            "--domain-id", str(ros.domain_id),
            "--localhost-only", "1" if ros.localhost_only else "0",
            "--rosbridge-port", str(ros.rosbridge_port),
            "--timeout", str(timeout_s),
        ]
        if self.in_dir is not None:
            cmd += ["--in-dir", str(self.in_dir)]
        for name, value in args.items():
            cmd += ["--launch-arg", f"{name}:={value}"]
        result = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=timeout_s + 60
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"start_ros2.sh exited {result.returncode}:\n{result.stdout}{result.stderr}"
            )

    def stop(self) -> None:
        cmd = [str(SCRIPTS / "stop_ros2.sh"), "--name", self.name, "--log-file", str(self.log_file)]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)
        except Exception:  # teardown must not hide the test's own failure
            log.exception("stop_ros2.sh failed for %s", self.name)

    def logs(self) -> str:
        result = subprocess.run(["docker", "logs", self.name], check=False, capture_output=True,
                                text=True, timeout=30)
        # The launch writes to both streams; stdout first is good enough for a search.
        return result.stdout + result.stderr

    async def wait_for_log_line(self, line: str, timeout_s: float) -> None:
        async def logged():
            return line in await asyncio.to_thread(self.logs)

        try:
            await wait_until(logged, timeout_s, f"{self.name} logs {line!r}",
                             poll_interval_s=LOG_POLL_S)
        except FlightAssertionError:
            tail = (await asyncio.to_thread(self.logs)).splitlines()[-50:]
            raise FlightAssertionError(
                f"{self.name} logs {line!r} within {timeout_s} s",
                "last log lines:\n" + "\n".join(tail)) from None
