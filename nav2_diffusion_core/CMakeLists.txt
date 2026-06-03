cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/trajectory.cpp
  src/scoring.cpp
  src/fan_rollout_model.cpp
  src/path.cpp
  src/fan_path_model.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ${PROJECT_NAME})

  ament_add_gtest(test_scoring test/test_scoring.cpp)
  target_link_libraries(test_scoring ${PROJECT_NAME})

  ament_add_gtest(test_fan_rollout_model test/test_fan_rollout_model.cpp)
  target_link_libraries(test_fan_rollout_model ${PROJECT_NAME})

  ament_add_gtest(test_path test/test_path.cpp)
  target_link_libraries(test_path ${PROJECT_NAME})

  ament_add_gtest(test_fan_path_model test/test_fan_path_model.cpp)
  target_link_libraries(test_fan_path_model ${PROJECT_NAME})
endif()

ament_package()
