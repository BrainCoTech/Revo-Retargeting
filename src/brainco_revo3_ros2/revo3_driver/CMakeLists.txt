# =============================================================================
# revo3_driver – CMakeLists.txt
# BrainCo Revo3 (21-DoF) RS-485/Modbus-only hardware interface for ros2_control
# Requires BC Revo3 SDK v1.0.4+ (run scripts/download_sdk.sh first)
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(revo3_driver)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# BC Revo3 SDK (vendor dir, populated by scripts/download_sdk.sh)
# -----------------------------------------------------------------------------
set(BRAINCO_SDK_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(BRAINCO_SDK_INCLUDE_DIR "${BRAINCO_SDK_VENDOR_DIR}/dist/include")
set(BRAINCO_SDK_LIB_DIR     "${BRAINCO_SDK_VENDOR_DIR}/dist/shared/linux")
set(BRAINCO_SDK_LIB_PATH    "${BRAINCO_SDK_LIB_DIR}/libbc_revo3_sdk.so")

if(NOT EXISTS "${BRAINCO_SDK_INCLUDE_DIR}/stark-sdk.h")
  message(FATAL_ERROR
    "stark-sdk.h not found in ${BRAINCO_SDK_INCLUDE_DIR}\n"
    "Run:  bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/download_sdk.sh")
endif()

if(NOT EXISTS "${BRAINCO_SDK_LIB_PATH}")
  message(FATAL_ERROR
    "libbc_revo3_sdk.so not found at ${BRAINCO_SDK_LIB_PATH}\n"
    "Run:  bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/download_sdk.sh")
endif()

add_library(bc_revo3_sdk SHARED IMPORTED GLOBAL)
set_target_properties(bc_revo3_sdk PROPERTIES
  IMPORTED_LOCATION "${BRAINCO_SDK_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${BRAINCO_SDK_INCLUDE_DIR}"
)

# -----------------------------------------------------------------------------
# ROS 2 dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)

# -----------------------------------------------------------------------------
# Driver library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/revo3_hand_api.cpp
  src/revo3_hand_hardware.cpp
  src/modbus_session.cpp
  src/sdk_helpers.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

target_link_libraries(${PROJECT_NAME}
  bc_revo3_sdk
)

# RPATH: find vendor .so at build time and the installed copy at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH   "${BRAINCO_SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

# -----------------------------------------------------------------------------
# Plugin registration
# -----------------------------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface revo3_driver_plugins.xml)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${BRAINCO_SDK_INCLUDE_DIR}/" DESTINATION include)

install(FILES "${BRAINCO_SDK_LIB_PATH}" DESTINATION lib)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES revo3_driver_plugins.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# -----------------------------------------------------------------------------
# Exports
# -----------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(pluginlib rclcpp rclcpp_lifecycle hardware_interface)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mit_gains test/test_mit_gains.cpp)
  target_include_directories(test_mit_gains PRIVATE include)
endif()

ament_package()
