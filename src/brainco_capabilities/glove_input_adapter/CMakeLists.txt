cmake_minimum_required(VERSION 3.8)
project(glove_input_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(humandex_pose_adapter STATIC
  src/humandex_pose_adapter.cpp
)
target_compile_features(humandex_pose_adapter PUBLIC cxx_std_17)
target_include_directories(humandex_pose_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(humandex_pose_adapter
  geometry_msgs
  manus_ros2_msgs
)

add_executable(humandex_pose_adapter_node
  src/humandex_pose_adapter_node.cpp
)
target_compile_features(humandex_pose_adapter_node PUBLIC cxx_std_17)
target_link_libraries(humandex_pose_adapter_node humandex_pose_adapter)
ament_target_dependencies(humandex_pose_adapter_node
  geometry_msgs
  manus_ros2_msgs
  rclcpp
  sensor_msgs
)

add_executable(humandex_absolute_revo2_node
  src/humandex_absolute_revo2_node.cpp
)
target_compile_features(humandex_absolute_revo2_node PUBLIC cxx_std_17)
target_link_libraries(humandex_absolute_revo2_node humandex_pose_adapter)
ament_target_dependencies(humandex_absolute_revo2_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  humandex_pose_adapter
  humandex_pose_adapter_node
  humandex_absolute_revo2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_humandex_pose_adapter test/test_humandex_pose_adapter.cpp)
  if(TARGET test_humandex_pose_adapter)
    target_link_libraries(test_humandex_pose_adapter humandex_pose_adapter)
    ament_target_dependencies(test_humandex_pose_adapter geometry_msgs manus_ros2_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs manus_ros2_msgs sensor_msgs)
ament_package()
