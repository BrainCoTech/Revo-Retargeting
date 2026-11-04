cmake_minimum_required(VERSION 3.8)
project(revo3_mit_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(revo3_mit_controller_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME} SHARED
  src/revo3_mit_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  revo3_mit_controller_msgs
)

pluginlib_export_plugin_description_file(controller_interface revo3_mit_controller.xml)

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
  FILES revo3_mit_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()