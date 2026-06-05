cmake_minimum_required(VERSION 3.8)
project(nav2_rrt_planner)

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
  nav2_util
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/rrt_star_planner.cpp
  src/rrt_connect_planner.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_rrt_planner_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES nav2_rrt_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rrt_star_planner
    test/test_rrt_star_planner.cpp
    TIMEOUT 120)
  target_link_libraries(test_rrt_star_planner ${PROJECT_NAME})
  ament_target_dependencies(test_rrt_star_planner ${dependencies})

  ament_add_gtest(test_rrt_connect_planner
    test/test_rrt_connect_planner.cpp
    TIMEOUT 120)
  target_link_libraries(test_rrt_connect_planner ${PROJECT_NAME})
  ament_target_dependencies(test_rrt_connect_planner ${dependencies})
endif()

ament_package()
