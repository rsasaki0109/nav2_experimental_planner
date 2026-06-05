cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_diffusion_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/candidate_markers.cpp
  src/safety_state_marker.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} visualization_msgs geometry_msgs nav2_diffusion_msgs)

add_executable(candidate_markers src/candidate_markers_node.cpp)
target_link_libraries(candidate_markers ${PROJECT_NAME})
ament_target_dependencies(candidate_markers rclcpp visualization_msgs nav2_diffusion_msgs)

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
install(
  TARGETS candidate_markers
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(visualization_msgs geometry_msgs nav2_diffusion_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_candidate_markers test/test_candidate_markers.cpp)
  target_link_libraries(test_candidate_markers ${PROJECT_NAME})
  ament_target_dependencies(test_candidate_markers visualization_msgs nav2_diffusion_msgs)

  ament_add_gtest(test_safety_state_marker test/test_safety_state_marker.cpp)
  target_link_libraries(test_safety_state_marker ${PROJECT_NAME})
  ament_target_dependencies(test_safety_state_marker visualization_msgs nav2_diffusion_msgs)
endif()

ament_package()
