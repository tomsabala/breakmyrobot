cmake_minimum_required(VERSION 3.8)
project(ral_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FaultEvent.msg
  msg/MissionState.msg
  srv/SpawnObstacle.srv
  srv/ClearObstacle.srv
  srv/SetScanDegradation.srv
  srv/SetMobilityFault.srv
  srv/Kidnap.srv
  srv/StartMission.srv
  DEPENDENCIES builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
