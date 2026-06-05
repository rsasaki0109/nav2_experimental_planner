cmake_minimum_required(VERSION 3.8)
project(nav2_planner_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(dependencies
  ament_index_cpp
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

# The curated learned models (model_zoo) are installed so the benchmarks can load
# them at runtime: Mode B (planner_benchmark / OnnxPathModel) and Mode A
# (controller_benchmark / OnnxTrajectoryModel).
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_global/costmap_flow.onnx
  DESTINATION share/${PROJECT_NAME}/models
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_global_transformer/costmap_transformer.onnx
  DESTINATION share/${PROJECT_NAME}/models
  RENAME diffusion_global_costmap_transformer.onnx
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_global_recurrent/costmap_recurrent.onnx
  DESTINATION share/${PROJECT_NAME}/models
  RENAME diffusion_global_costmap_recurrent.onnx
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_local/costmap_flow.onnx
  DESTINATION share/${PROJECT_NAME}/models
  RENAME diffusion_local_costmap_flow.onnx
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_local_transformer/costmap_transformer.onnx
  DESTINATION share/${PROJECT_NAME}/models
  RENAME diffusion_local_costmap_transformer.onnx
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../model_zoo/diffusion_local_recurrent/costmap_recurrent.onnx
  DESTINATION share/${PROJECT_NAME}/models
  RENAME diffusion_local_costmap_recurrent.onnx
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
