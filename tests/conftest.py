"""Fixtures: one fresh PX4 container, mavsdk_server and recorder per test."""

from __future__ import annotations

import json
import re
import time

import pytest
import pytest_asyncio

from framework.config import load_baseline_params, load_config
from framework.mission import load_scenario
from framework.sitl import (
    ARTIFACTS,
    MavsdkServer,
    SitlInstance,
    assert_params,
    connect,
    scaled_loss_params,
    wait_ready,
)
from framework.telemetry_log import TelemetryRecorder


@pytest.fixture(scope="session")
def cfg():
    return load_config()


@pytest.fixture
def test_id(request) -> str:
    # Node IDs contain ":" and "[", which break Docker bind mounts and artifact upload.
    return re.sub(r"[^A-Za-z0-9_.-]", "_", request.node.name)


@pytest.fixture
def scenario(request):
    """The scenario from indirect parametrization or the ``scenario`` marker, else None."""
    if hasattr(request, "param"):
        return load_scenario(request.param)
    marker = request.node.get_closest_marker("scenario")
    return load_scenario(marker.args[0]) if marker else None


@pytest.fixture
def boot_params(cfg, scenario) -> dict:
    # Tests without a scenario boot with the baseline alone.
    params = scenario.px4_params if scenario else load_baseline_params()
    return scaled_loss_params(params, cfg)


@pytest.fixture
def sitl(cfg, test_id, boot_params, request):
    instance = SitlInstance(cfg, test_id, boot_params)
    request.node.boot_started = time.monotonic()
    instance.start()
    yield instance
    instance.stop()


@pytest.fixture
def mavsdk_server(cfg, sitl, test_id):
    server = MavsdkServer(cfg, ARTIFACTS / "logs" / f"{test_id}.mavsdk_server.log")
    server.start()
    yield server
    server.stop()


@pytest_asyncio.fixture
async def drone(cfg, mavsdk_server, boot_params, request):
    system = await connect(cfg, mavsdk_server)
    await assert_params(system, boot_params, cfg)
    await wait_ready(system, cfg)
    request.node.boot_to_ready_s = time.monotonic() - request.node.boot_started
    return system


@pytest_asyncio.fixture
async def recorder(cfg, drone, test_id):
    rec = TelemetryRecorder(drone, cfg, test_id)
    await rec.start()
    yield rec
    await rec.stop()


@pytest.hookimpl(wrapper=True)
def pytest_runtest_makereport(item, call):
    report = yield
    rec = item.funcargs.get("recorder") if hasattr(item, "funcargs") else None
    if report.when == "call" and report.failed and rec is not None:
        text = f"telemetry log: {rec.path}\nlast snapshot:\n{json.dumps(rec.snapshot(), indent=2)}"
        # junitxml copies sections named "Captured stdout ..." into <system-out>.
        report.sections.append(("Captured stdout teardown", text))
    return report
