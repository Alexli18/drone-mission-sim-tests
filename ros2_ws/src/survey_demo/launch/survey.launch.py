"""Live launch: DDS agent, rosbridge and rosapi, bag recorder, the survey nodes and the
ready gate.

Started by ``framework.ros2.Ros2Instance`` (``scripts/start_ros2.sh``), which always
passes ``params``, ``out``, ``agent_port``, ``rosbridge_address`` and ``rosbridge_port``.
The ready gate prints ``SURVEY READY`` (``output="screen"``, so the line reaches
``docker logs``) once every node, service and subscription is in place and the toy
sensor and the georeferencing node have both received a PX4 pose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = {
    "params": "",
    "out": "/out",
    "agent_port": "8888",
    "rosbridge_address": "127.0.0.1",
    "rosbridge_port": "9090",
}
RECORDED = ["/survey/capture", "/survey/pose_used", "/survey/georef_capture"]


def generate_launch_description():
    params = LaunchConfiguration("params")
    bag_qos = PathJoinSubstitution([FindPackageShare("survey_demo"), "config", "bag_qos.yaml"])
    # 1. Listens on all interfaces: the agent has no bind option.
    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "--port", LaunchConfiguration("agent_port")],
        name="micro_xrce_agent", output="screen")
    # 2. rosbridge (bound to rosbridge_address) and rosapi.
    rosbridge = Node(
        package="rosbridge_server", executable="rosbridge_websocket",
        name="rosbridge_websocket", output="screen",
        parameters=[{
            "address": LaunchConfiguration("rosbridge_address"),
            "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
        }])
    rosapi = Node(package="rosapi", executable="rosapi_node", name="rosapi", output="screen")
    # 3. The recording the replay check plays back.
    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o",
             PathJoinSubstitution([LaunchConfiguration("out"), "bag"]),
             "--qos-profile-overrides-path", bag_qos, *RECORDED],
        name="bag_recorder", output="screen")
    # 4. The nodes, all from the run's parameter file.
    nodes = [
        Node(package="survey_demo", executable="toy_sensor_node", name="toy_sensor_node",
             output="screen", parameters=[params]),
        Node(package="sensor_bridge", executable="georeference_node", name="georeference_node",
             output="screen", parameters=[params]),
        Node(package="survey_demo", executable="marker_detector_node",
             name="marker_detector_node", output="screen", parameters=[params]),
    ]
    # 5. The ready gate (live lists from the parameter file, exit_when_ready false).
    gate = Node(package="survey_demo", executable="ready_gate", name="ready_gate",
                output="screen", parameters=[params])
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in ARGS.items()),
        agent, rosbridge, rosapi, recorder, *nodes, gate,
    ])
