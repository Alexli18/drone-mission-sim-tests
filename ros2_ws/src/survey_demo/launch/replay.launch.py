"""Replay launch: a live run's recording through a fresh georeferencing node and
detector, without PX4, the DDS agent or the toy sensor.

Started by ``framework.ros2.Ros2Instance`` (``scripts/start_ros2.sh``), which always
passes ``params``, ``out``, ``agent_port`` (declared and unused here), ``rosbridge_address``
and ``rosbridge_port``; the test adds ``bag`` (the live run's bag under ``/in``).
The ready gate (replay lists, ``exit_when_ready`` true) prints ``SURVEY READY`` once the
nodes subscribe; on its exit ``bag_player`` plays ``/survey/capture`` and
``/survey/pose_used`` at rate 1 once its publishers have matched the subscriptions
(``ros2 bag play --delay`` lost the first 0.77 s of the bag in CI), and on the player's
exit a ``LogInfo`` prints ``SURVEY REPLAY DONE`` (read by the test from ``docker logs``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    "params": "",
    "out": "/out",
    "bag": "/in/bag",
    "agent_port": "8888",
    "rosbridge_address": "127.0.0.1",
    "rosbridge_port": "9090",
}
PLAYED = ["/survey/capture", "/survey/pose_used"]
DONE_LINE = "SURVEY REPLAY DONE"


def generate_launch_description():
    params = LaunchConfiguration("params")
    rosbridge = Node(
        package="rosbridge_server", executable="rosbridge_websocket",
        name="rosbridge_websocket", output="screen",
        parameters=[{
            "address": LaunchConfiguration("rosbridge_address"),
            "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
        }])
    rosapi = Node(package="rosapi", executable="rosapi_node", name="rosapi", output="screen")
    # The nodes under test, with the replay parameters (pose_type sample, no republish).
    nodes = [
        Node(package="sensor_bridge", executable="georeference_node", name="georeference_node",
             output="screen", parameters=[params]),
        Node(package="survey_demo", executable="marker_detector_node",
             name="marker_detector_node", output="screen", parameters=[params]),
    ]
    gate = Node(package="survey_demo", executable="ready_gate", name="ready_gate",
                output="screen", parameters=[params])
    # Plays only after its publishers match the georeferencing node (one subscription
    # per topic), plus a settle time for the subscription side of the match.
    player = Node(package="survey_demo", executable="bag_player", name="bag_player",
                  output="screen",
                  parameters=[{"bag": LaunchConfiguration("bag"), "topics": PLAYED,
                               "min_subscribers": 1, "match_timeout_s": 30.0,
                               "settle_s": 1.0}])
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in ARGS.items()),
        rosbridge, rosapi, *nodes, gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[player])),
        RegisterEventHandler(OnProcessExit(target_action=player,
                                           on_exit=[LogInfo(msg=DONE_LINE)])),
    ])
