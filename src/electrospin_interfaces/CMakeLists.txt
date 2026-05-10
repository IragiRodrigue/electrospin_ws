cmake_minimum_required(VERSION 3.8)
project(electrospin_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FiberQuality.msg"
  "msg/CollectorStatus.msg"
  "msg/ElectrospinCommand.msg"
  "msg/SystemStatus.msg"
  "msg/HumanPose.msg"
  "msg/HandGesture.msg"
  "msg/MotionCommand.msg"
  "srv/SetRPM.srv"
  "srv/SetFlowRate.srv"
  "srv/SetDistance.srv"
  "action/OptimizeProcess.action"
  DEPENDENCIES std_msgs
)

ament_package()
