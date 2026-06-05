cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module holding the course specs + generator (importable by the launch
# as `from nav2_diffusion_sim import gen_courses`).
ament_python_install_package(${PROJECT_NAME})

# Course assets (generated from the specs by gen_courses.py) and the launch.
install(
  DIRECTORY worlds maps launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Geometry / artifact-consistency tests for the course generator (no ROS, no
  # Gazebo). The closed-loop run itself needs a real ROS host
  # (docs/simulation.md section 10.5).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gen_courses test/test_gen_courses.py)
endif()

ament_package()
