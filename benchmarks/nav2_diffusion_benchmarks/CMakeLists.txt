cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_diffusion_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/metrics.cpp
  src/collision_metrics.cpp
  src/scores.cpp
  src/report.cpp
  src/run_recorder.cpp
  src/scenario.cpp
  src/aggregate.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} nav2_diffusion_core nav2_costmap_2d geometry_msgs)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(benchmark_runner src/benchmark_runner_node.cpp)
target_link_libraries(benchmark_runner ${PROJECT_NAME})
ament_target_dependencies(benchmark_runner rclcpp nav_msgs std_srvs)
install(
  TARGETS benchmark_runner
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(nav2_diffusion_core nav2_costmap_2d geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics test/test_metrics.cpp)
  target_link_libraries(test_metrics ${PROJECT_NAME})

  ament_add_gtest(test_collision_metrics test/test_collision_metrics.cpp)
  target_link_libraries(test_collision_metrics ${PROJECT_NAME})
  ament_target_dependencies(test_collision_metrics nav2_costmap_2d geometry_msgs)

  ament_add_gtest(test_scores test/test_scores.cpp)
  target_link_libraries(test_scores ${PROJECT_NAME})

  ament_add_gtest(test_run_recorder test/test_run_recorder.cpp)
  target_link_libraries(test_run_recorder ${PROJECT_NAME})

  ament_add_gtest(test_scenario test/test_scenario.cpp)
  target_link_libraries(test_scenario ${PROJECT_NAME})

  ament_add_gtest(test_aggregate test/test_aggregate.cpp)
  target_link_libraries(test_aggregate ${PROJECT_NAME})

  ament_add_gtest(test_report test/test_report.cpp)
  target_link_libraries(test_report ${PROJECT_NAME})
endif()

ament_package()
