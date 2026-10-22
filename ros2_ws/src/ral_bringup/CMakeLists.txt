cmake_minimum_required(VERSION 3.8)
project(ral_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The bridge config is the one place the truth-vs-estimate rule is enforced,
  # so it is checked without a simulator: this runs in any build.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge_contract test/test_bridge_contract.py)
endif()

ament_package()
