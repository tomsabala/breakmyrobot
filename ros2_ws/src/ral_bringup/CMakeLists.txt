cmake_minimum_required(VERSION 3.8)
project(ral_bringup)

find_package(ament_cmake REQUIRED)

# `maps` holds the committed static map and the measured world->map alignment
# that belongs to it; both are runtime inputs resolved through the share
# directory, so they must be installed, not just tracked.
install(DIRECTORY
  launch
  config
  maps
  # `rviz` is loaded by the HOST's rviz2 straight from the source tree (the
  # container image has no rviz2), but it is installed too so a deployed
  # share directory is self-contained.
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The bridge config is the one place the truth-vs-estimate rule is enforced,
  # so it is checked without a simulator: this runs in any build.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge_contract test/test_bridge_contract.py)

  # Same idea for the navigation configuration: Nav2's parameters are copied
  # from a TurtleBot3 reference, and the defaults that are wrong for this
  # robot (base_footprint, 100 m laser range, a 0.26 m/s smoother clamp) all
  # fail quietly at runtime. This catches them at build time.
  ament_add_pytest_test(test_nav2_contract test/test_nav2_contract.py)
endif()

ament_package()
