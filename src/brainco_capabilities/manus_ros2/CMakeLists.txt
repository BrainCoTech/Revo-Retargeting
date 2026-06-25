cmake_minimum_required(VERSION 3.11)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

option(MANUS_USE_CORE_SDK "Link against libManusSDK.so for MANUS Core connection mode" OFF)
if(MANUS_USE_CORE_SDK)
  set(MANUS_SDK_TARGET ManusSDK)
  set(MANUS_SDK_LIBRARY_FILE libManusSDK.so)
else()
  set(MANUS_SDK_TARGET ManusSDK_Integrated)
  set(MANUS_SDK_LIBRARY_FILE libManusSDK_Integrated.so)
endif()

add_executable(manus_data_publisher 
  src/manus_data_publisher.cpp
  src/CalibrationFiles.cpp
  src/ClientPlatformSpecific.cpp
  src/ManusDataPublisher.cpp)
target_include_directories(manus_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ./ManusSDK/include)
target_link_directories(manus_data_publisher PRIVATE ./ManusSDK/lib/)

#Use the following line if you are using the integrated version of the SDK and haven't installed the GRPC / Protobuf dependencies needed for remote connection
target_link_libraries(manus_data_publisher ${MANUS_SDK_TARGET} ncurses)
#target_link_libraries(manus_data_publisher ManusSDK ncurses)

target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(manus_calibration_tool
  src/manus_calibration_tool.cpp
  src/CalibrationFiles.cpp)
target_include_directories(manus_calibration_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ./ManusSDK/include)
target_link_directories(manus_calibration_tool PRIVATE ./ManusSDK/lib/)
target_link_libraries(manus_calibration_tool ${MANUS_SDK_TARGET} ncurses)
target_compile_features(manus_calibration_tool PUBLIC c_std_99 cxx_std_17)

install(TARGETS manus_data_publisher manus_calibration_tool
  DESTINATION lib/${PROJECT_NAME})

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${MANUS_LINUX_PATH}./ManusSDK/lib)

#Use the following line if you are using the integrated version of the SDK and haven't installed the GRPC / Protobuf dependencies needed for remote connection
set(LIBRARY_FILE ${LIBRARY_DIR}/${MANUS_SDK_LIBRARY_FILE})
#set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK.so)

install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})

set_target_properties(manus_data_publisher manus_calibration_tool PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs ament_index_cpp)
ament_target_dependencies(manus_calibration_tool ament_index_cpp)

install(DIRECTORY
  calibrations/
  DESTINATION share/${PROJECT_NAME}/calibrations
  FILES_MATCHING PATTERN "*.mcal"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
