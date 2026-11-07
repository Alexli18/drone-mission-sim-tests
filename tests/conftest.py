"""Fixtures: one fresh PX4 container, mavsdk_server and recorder per test.

``line_survey`` scenarios add the ROS container and the rosbridge client.
``framework.ros2`` and ``framework.rosbridge`` are imported inside those fixtures only,
so ``unit``, ``sitl`` and spike runs never import them.
"""

from __future__ import annotations

import dataclasses
import json
import os
import re
import time

import pytest
import pytest_asyncio

from framework import reporting
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
def base_cfg():
    return load_config()


@pytest.fixture
def cfg(base_cfg, scenario):
    """The configuration; ROS runs use ``px4.speed_factor_ros`` (2-vCPU runner)."""
    if not _is_ros(scenario):
        return base_cfg
    px4 = dataclasses.replace(base_cfg.px4, speed_factor=base_cfg.px4.speed_factor_ros)
    return dataclasses.replace(base_cfg, px4=px4)


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


@pytest.fixture(autouse=True)
def allure_identity(request, base_cfg, test_id):
    """Flight tests: the Allure parameters and description (framework/reporting.py)."""
    flight = request.node.get_closest_marker("sitl") is not None
    if flight:
        reporting.apply_parameters(base_cfg, request.getfixturevalue("scenario"))
    yield
    if flight:
        reporting.apply_description(test_id)


def _is_ros(scenario) -> bool:
    return scenario is not None and scenario.survey is not None


@pytest.fixture
def boot_params(cfg, scenario) -> dict:
    # Tests without a scenario boot with the baseline alone.
    params = scenario.px4_params if scenario else load_baseline_params()
    return scaled_loss_params(params, cfg)


@pytest.fixture
def sitl(cfg, test_id, boot_params, scenario, request):
    if _is_ros(scenario):
        # rcS sets UXRCE_DDS_DOM_ID from ROS_DOMAIN_ID after the PX4_PARAM_* overrides,
        # so the DDS domain goes in through the environment (docs/qa-notes.md, "DDS").
        instance = SitlInstance(cfg, test_id, boot_params,
                                env={"ROS_DOMAIN_ID": str(cfg.ros.domain_id)}, dds_own_ip=True)
    else:
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
async def drone(cfg, mavsdk_server, boot_params, scenario, request):
    system = await connect(cfg, mavsdk_server)
    checked = dict(boot_params)
    if _is_ros(scenario):
        checked["UXRCE_DDS_DOM_ID"] = cfg.ros.domain_id  # from ROS_DOMAIN_ID, see sitl
    await assert_params(system, checked, cfg)
    await wait_ready(system, cfg)
    request.node.boot_to_ready_s = time.monotonic() - request.node.boot_started
    return system


@pytest_asyncio.fixture
async def recorder(cfg, drone, test_id):
    rec = TelemetryRecorder(drone, cfg, test_id)
    await rec.start()
    yield rec
    await rec.stop()


@pytest.fixture
def ros2(cfg, scenario, sitl, test_id):
    """The live ROS container (after PX4: ``sitl`` comes first)."""
    from framework.ros2 import Ros2Instance
    from framework.survey import node_params

    seed_offset = int(os.environ.get("SURVEY_SEED_OFFSET", "0"))
    params = node_params(scenario, cfg, seed_offset, "live",
                         finalize_timeout_s=cfg.timeouts_wall_s.finalize)
    instance = Ros2Instance(cfg, test_id, name_suffix="live", launch="survey_demo survey.launch.py",
                            out_dir=ARTIFACTS / "ros" / test_id / "live", params=params)
    try:
        instance.start()
        yield instance
    finally:
        instance.stop()


@pytest_asyncio.fixture
async def rosbridge(cfg, ros2, test_id):
    from framework.rosbridge import RosbridgeClient

    client = RosbridgeClient(cfg, test_id, ARTIFACTS / "ros" / test_id / "recordings" / "live")
    try:
        await client.connect()
        yield client
    finally:
        client.close()


@pytest.hookimpl(wrapper=True)
def pytest_runtest_makereport(item, call):
    report = yield
    rec = item.funcargs.get("recorder") if hasattr(item, "funcargs") else None
    if report.when == "call" and report.failed and rec is not None:
        text = f"telemetry log: {rec.path}\nlast snapshot:\n{json.dumps(rec.snapshot(), indent=2)}"
        # junitxml copies sections named "Captured stdout ..." into <system-out>.
        report.sections.append(("Captured stdout teardown", text))
    if report.when == "call" and item.get_closest_marker("sitl") is not None:
        reporting.attach_metrics(item.user_properties)
        if report.failed and rec is not None:
            reporting.attach_failure(rec)
    return report
