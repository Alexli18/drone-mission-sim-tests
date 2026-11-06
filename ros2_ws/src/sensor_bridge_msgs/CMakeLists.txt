cmake_minimum_required(VERSION 3.8)
project(sensor_bridge_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorCapture.msg"
  "msg/PoseSample.msg"
  "msg/GeorefCapture.msg"
  "msg/SurveyStatus.msg"
  "msg/Marker.msg"
  "msg/MarkerList.msg"
  "srv/Stop.srv"
  "srv/Flush.srv"
  "srv/Finalize.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
