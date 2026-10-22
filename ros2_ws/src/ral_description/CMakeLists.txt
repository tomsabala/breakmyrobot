cmake_minimum_required(VERSION 3.8)
project(ral_description)

find_package(ament_cmake REQUIRED)

# Data-only package. `models` is also what GZ_SIM_RESOURCE_PATH must point at
# (the directory *containing* model directories) for `model://ral_roomba` to
# resolve; ral_bringup's launch file sets that from this install location.
install(DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
