cmake_minimum_required(VERSION 3.8)
project(manus_revo3_retarget)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)
find_package(revo3_mit_controller_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME}_algorithms
  src/four_finger_retarget.cpp
  src/spread_retarget.cpp
)
target_compile_features(${PROJECT_NAME}_algorithms PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_algorithms
  Eigen3::Eigen
)

add_library(${PROJECT_NAME}_thumb_pinocchio SHARED
  src/thumb_retarget.cpp
)
target_compile_features(${PROJECT_NAME}_thumb_pinocchio PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_thumb_pinocchio PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_thumb_pinocchio
  Eigen3::Eigen
  pinocchio::pinocchio
)

add_executable(retarget_node src/retarget_node_cpp.cpp)
target_compile_features(retarget_node PUBLIC cxx_std_17)
target_link_libraries(retarget_node ${PROJECT_NAME}_algorithms ${CMAKE_DL_LIBS})
ament_target_dependencies(retarget_node
  rclcpp
  std_msgs
  manus_ros2_msgs
  revo3_mit_controller_msgs
  ament_index_cpp
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_algorithms
  ${PROJECT_NAME}_thumb_pinocchio
  retarget_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS manus_revo3_retarget/command_state_viewer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_state_viewer
)

install(PROGRAMS manus_revo3_retarget/joint_state_aligner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_aligner
)

install(PROGRAMS manus_revo3_retarget/manus_record.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manus_record
)

install(PROGRAMS manus_revo3_retarget/manus_replay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manus_replay
)

install(PROGRAMS manus_revo3_retarget/quintic_joint_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME quintic_joint_test
)

install(PROGRAMS manus_revo3_retarget/retarget_tuning_panel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME retarget_tuning_panel
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
