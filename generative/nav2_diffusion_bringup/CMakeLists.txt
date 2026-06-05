cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params foxglove config
  DESTINATION share/${PROJECT_NAME}
)

# In-launch mission node for the headless Gazebo closed-loop benchmark.
install(
  PROGRAMS scripts/sim_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit-test the pure (no-ROS) helpers of the mission harness: leg-spec parsing,
  # metric aggregation, leaderboard formatting. The closed-loop drive itself needs
  # a live Nav2 + Gazebo (docs/simulation.md section 10.5).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_mission test/test_sim_mission.py)
endif()

ament_package()
