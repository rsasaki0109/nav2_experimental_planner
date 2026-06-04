cmake_minimum_required(VERSION 3.8)
project(nav2_planner_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(planner_benchmark src/planner_benchmark.cpp)
target_compile_features(planner_benchmark PUBLIC cxx_std_17)
ament_target_dependencies(planner_benchmark ${dependencies})

add_executable(controller_benchmark src/controller_benchmark.cpp)
target_compile_features(controller_benchmark PUBLIC cxx_std_17)
ament_target_dependencies(controller_benchmark ${dependencies})

install(
  TARGETS planner_benchmark controller_benchmark
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
