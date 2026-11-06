# Testing the PX4 to ROS 2 bridge

How the ROS 2 side of the line-survey test is wired, and the places where it broke.
Everything here was found by running the stack in CI and on a Mac, not by reading
documentation; the matching test or check is named for each item.

## Setup

* **Transport:** PX4's `uxrce_dds_client` (started by PX4's `rcS`) talks UDP to a
  Micro XRCE-DDS Agent (`MicroXRCEAgent udp4 --port 8888`) in the ROS container. Both
  containers use host networking. The agent is built from a pinned tag and commit in
  `docker/ros2-survey-base/Dockerfile`, together with `px4_msgs` at the commit that
  matches the pinned PX4 (`config.yaml`, `px4.image`); a mismatch of the message
  definitions shows up as no pose or as garbage, so both are pinned together.
* **Topics read:** `/fmu/out/vehicle_local_position_v1` (the `_v1` suffix is part of the
  name in PX4 v1.18.0-rc1; 50 Hz of simulated time, best effort) and
  `/fmu/out/home_position_v1` (best effort, transient local: published on change, so a
  node that starts late still gets the current home). The home position's `z` is the
  ground reference that heights are measured from (PX4's local origin is not the ground).
* **Test access:** `rosbridge_websocket` plus `rosapi` in the same container. The test
  never imports ROS: it talks JSON over a websocket (`framework/rosbridge.py`, built on
  `roslibpy`), subscribes to topics, calls services and records every message it
  subscribes to as JSON lines.
* **Readiness:** the `ready_gate` node waits until every node, service and subscription
  exists and both the sensor and the georeferencing node have a PX4 pose, then prints
  `SURVEY READY` and publishes `/survey/ready`. `scripts/start_ros2.sh` waits for that line
  in `docker logs` and for the rosbridge port. The sensor does not emit before
  `/survey/ready`, so no capture is produced before the graph is complete.

## Gotchas, each with its guard

| Gotcha | What happens | Guard |
|---|---|---|
| The DDS domain is set by `ROS_DOMAIN_ID` in the environment of the PX4 container, not by the `UXRCE_DDS_DOM_ID` boot parameter | `rcS` copies `ROS_DOMAIN_ID` into the parameter *after* the `PX4_PARAM_*` overrides, so a boot parameter is silently overwritten and the nodes see nothing | `start_sitl.sh --env`; the scenario loader rejects `UXRCE_DDS_DOM_ID` in `px4_params`; the `drone` fixture reads the parameter back and compares it with the domain (`tests/unit/test_mission.py`, `tests/conftest.py`) |
| `UXRCE_DDS_SYNCT` must be 0 in simulation | With the default (time synchronization on) the stamps are Unix wall time, not monotonic, and jump back to boot time. With 0 they are PX4's simulated time, monotonic and inside MAVSDK's simulated time | Every `line_survey` scenario sets it; `test_the_scenario_matrix` asserts it. A stated divergence from a real vehicle |
| `ROS_LOCALHOST_ONLY=1` hides the agent's topics | The agent ignores the setting; with it on, Humble nodes in the same container discover no `/fmu` topics | `ros.localhost_only: false` in `config.yaml`, passed by `start_ros2.sh` |
| Docker Desktop (macOS): the image's entrypoint points the DDS client at `host.docker.internal` | UDP replies over 127.0.0.1 are lost there, so the client never connects. CI (Linux, host network) is unaffected | `start_sitl.sh --dds-own-ip` patches the client's `-h` to the container's own address; a no-op on Linux |
| Pose subscriptions with a small queue lose poses | See "Pose loss" below | `PX4_DEPTH = 500` in `georeference_node.py`; `usable_without_truth` and the coverage agreement rule |
| `roslibpy` runs one Twisted reactor per process, which cannot be restarted | The second connection of a pytest session never opened | See "The rosbridge client" below |
| Humble parameter files cannot hold an empty list | A scenario without markers failed to start the sensor | `node_params` leaves the marker lists out; `declare_optional` reads them as empty (`tests/unit/test_survey.py`) |

## Pose loss from the QoS depth

PX4 publishes every pose (the ULog has them all); they were lost on the ROS side. The
first version subscribed best effort with keep-last depth 5. At a PX4 speed factor of
about 6 the 50 Hz poses arrive at about 300 Hz of wall time, so depth 5 is about 16 ms,
and a single-threaded node busy with one capture for longer than that loses the oldest
poses. In a CI run the sensor lost 34 % of the poses and the georeferencing node 8 %, so
the two disagreed about the coverage (0.55 against 0.88). Fixes: keep-last 500 (about
1.7 s of wall time, a pose is about 200 bytes), the sensor's pose callback only appends
the pose and a 10 ms timer advances the sensor, and the sensor interpolates across wider
pose gaps than the node under test (0.5 s against 0.2 s). Guards: the test compares the
coverage of the two nodes (they use different pose streams), requires that no usable
capture was built without a pose, and records `no_pose` and the pose counters.

## The rosbridge client (`framework/rosbridge.py`)

`roslibpy` is built on Twisted, whose reactor cannot be restarted. The client therefore
starts the reactor once per process in a daemon thread, never calls `Ros.terminate()`
and only closes connections. Twisted is not thread-safe: a `Ros` created on another
thread while the reactor already runs queues the TCP connect without waking the reactor,
so the first connection of a session worked and the second and later ones never opened.
The fix creates and closes every `Ros` on the reactor thread
(`threads.blockingCallFromThread`), and blocking calls run in `asyncio.to_thread`.

`tests/unit/test_rosbridge.py` tests this against a fake websocket server in the test
process, with no ROS: two sequential connections must both open and be created on the
reactor thread; a connect to a closed port must raise within the timeout and leave the
reactor running. The fake server had its own race, found under `pytest -n 2`: it counted
the connection after sending the handshake answer, so a fast client could connect, close
and assert before the server thread ran again. The server now counts first.

## Replay (`bag_player`)

The live run records `/survey/capture` and `/survey/pose_used` (the poses each decision
used, republished once each with the ground reference of their release). The replay
starts fresh nodes and plays the bag. `ros2 bag play --delay 2.0` was replaced: with
reliable, volatile QoS a message published before the subscription has matched is lost,
and a fixed delay is a race. In a CI run the first 0.77 s of the bag never reached the
replayed georeferencing node (captures 0 to 55 and the 89 poses before them), so the
replay "disagreed" about captures it had never seen. `bag_player` publishes only after
each of its publishers has matched at least one subscription, then waits a settle time
for the subscription side to match; it exits 1 if nothing matches in time.

A second replay detail: in a replay the node's notion of "now" is the newest
*republished* pose, which can pass a held capture's arrival by the hold timeout before
the capture's own poses arrive. The replay therefore runs without the simulated-time hold
timeout (`replay_params`), and the check compares only captures from takeoff on.
