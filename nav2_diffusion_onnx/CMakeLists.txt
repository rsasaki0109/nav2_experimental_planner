cmake_minimum_required(VERSION 3.8)
project(nav2_diffusion_onnx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_diffusion_core REQUIRED)
find_package(pluginlib REQUIRED)

# Optional backend: build only when onnxruntime is discoverable (onnxruntime_vendor,
# a system install, or CMAKE_PREFIX_PATH / onnxruntime_DIR pointing at its config).
# Otherwise produce an empty package so a plain `colcon build` never fails for
# contributors without onnxruntime.
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  message(STATUS "nav2_diffusion_onnx: onnxruntime not found; skipping optional backend")
  ament_package()
  return()
endif()

# The C++ headers can sit at the include root or under core/session depending on
# the distribution; locate onnxruntime_cxx_api.h robustly.
get_target_property(_ort_include onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
find_path(ONNXRUNTIME_CXX_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${_ort_include}
  PATH_SUFFIXES "" core/session onnxruntime/core/session)
if(NOT ONNXRUNTIME_CXX_INCLUDE_DIR)
  message(FATAL_ERROR "onnxruntime_cxx_api.h not found near ${_ort_include}")
endif()

# Resolve the onnxruntime library directory for rpath and test LD_LIBRARY_PATH.
get_target_property(_ort_loc onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)
if(NOT _ort_loc)
  get_target_property(_ort_loc onnxruntime::onnxruntime IMPORTED_LOCATION)
endif()
get_filename_component(ONNXRUNTIME_LIB_DIR "${_ort_loc}" DIRECTORY)

add_library(${PROJECT_NAME} SHARED
  src/onnx_trajectory_model.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${ONNXRUNTIME_CXX_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} nav2_diffusion_core pluginlib)
target_link_libraries(${PROJECT_NAME} onnxruntime::onnxruntime)

pluginlib_export_plugin_description_file(nav2_diffusion_core nav2_diffusion_onnx_plugin.xml)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}")

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
  FILES nav2_diffusion_onnx_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(nav2_diffusion_core pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Generate a tiny fixture model with PyTorch at build time (kept out of git).
  set(_test_model "${CMAKE_CURRENT_BINARY_DIR}/test_model.onnx")
  add_custom_command(
    OUTPUT "${_test_model}"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONHOME=" python3
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_test_model.py" "${_test_model}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_test_model.py"
    COMMENT "Generating ONNX fixture model"
  )
  add_custom_target(generate_test_model DEPENDS "${_test_model}")

  ament_add_gtest(test_onnx_trajectory_model test/test_onnx_trajectory_model.cpp
    APPEND_LIBRARY_DIRS "${ONNXRUNTIME_LIB_DIR}")
  add_dependencies(test_onnx_trajectory_model generate_test_model)
  target_link_libraries(test_onnx_trajectory_model ${PROJECT_NAME})
  target_compile_definitions(test_onnx_trajectory_model
    PRIVATE ONNX_TEST_MODEL="${_test_model}")
endif()

ament_package()
