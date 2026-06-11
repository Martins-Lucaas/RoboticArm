cmake_minimum_required(VERSION 3.8)
project(touch_pack_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PalpationStart.msg"
  "msg/PalpationStatus.msg"
  "msg/SyncedTouch.msg"
  DEPENDENCIES std_msgs
)

ament_package()
