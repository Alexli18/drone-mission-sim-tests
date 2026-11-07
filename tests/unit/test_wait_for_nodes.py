"""wait_for_nodes: ask rosapi for node details only once it lists the nodes."""

import pytest

pytest.importorskip("roslibpy")

from framework.assertions import FlightAssertionError
from framework.rosbridge import wait_for_nodes

pytestmark = pytest.mark.unit

NODES = ("/georeference_node", "/marker_detector_node")


class FakeRosapi:
    """Answers ``/rosapi/nodes`` from a script: a list of node names, or an error text."""

    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    async def call_service(self, name, srv_type, request, timeout_s):
        self.calls.append((name, srv_type))
        step = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        if isinstance(step, str):
            raise FlightAssertionError(f"{name} succeeded", f"error: {step}")
        return {"nodes": step}


async def test_returns_at_once_when_both_nodes_are_listed():
    client = FakeRosapi([["/rosapi", *NODES]])
    await wait_for_nodes(client, NODES, timeout_s=1.0, poll_s=0.01)
    assert client.calls == [("/rosapi/nodes", "rosapi_msgs/srv/Nodes")]


async def test_waits_until_the_nodes_appear():
    client = FakeRosapi([["/rosapi"], ["/rosapi", NODES[0]], ["/rosapi", *NODES]])
    await wait_for_nodes(client, NODES, timeout_s=1.0, poll_s=0.01)
    assert len(client.calls) == 3


async def test_a_rosapi_that_has_not_started_is_tried_again():
    client = FakeRosapi(["Service /rosapi/nodes does not exist", ["/rosapi", *NODES]])
    await wait_for_nodes(client, NODES, timeout_s=1.0, poll_s=0.01)
    assert len(client.calls) == 2


async def test_a_node_that_never_appears_fails_and_names_it():
    client = FakeRosapi([["/rosapi", NODES[0]]])
    with pytest.raises(FlightAssertionError) as raised:
        await wait_for_nodes(client, NODES, timeout_s=0.1, poll_s=0.01)
    text = str(raised.value)
    assert "within 0.1 s" in text and "missing ['/marker_detector_node']" in text
    assert "/georeference_node" in text


async def test_a_rosapi_that_never_answers_fails_with_its_last_error():
    client = FakeRosapi(["Service /rosapi/nodes does not exist"])
    with pytest.raises(FlightAssertionError) as raised:
        await wait_for_nodes(client, NODES, timeout_s=0.1, poll_s=0.01)
    assert "listed None" in str(raised.value) and "does not exist" in str(raised.value)
