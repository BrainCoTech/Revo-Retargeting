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

set(MANUS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ManusSDK" CACHE PATH "Path to an unpacked MANUS SDK")
set(MANUS_SDK_INCLUDE_DIR "${MANUS_SDK_ROOT}/include")
set(MANUS_SDK_LIBRARY_DIR "${MANUS_SDK_ROOT}/lib")
find_library(MANUS_SDK_LIBRARY
  NAMES ManusSDK_Integrated ManusSDK
  PATHS "${MANUS_SDK_LIBRARY_DIR}"
  NO_DEFAULT_PATH)

set(MANUS_SDK_AVAILABLE FALSE)
set(MANUS_SDK_UNAVAILABLE_REASON "MANUS SDK was not found under ${MANUS_SDK_ROOT}.")
if(EXISTS "${MANUS_SDK_INCLUDE_DIR}/ManusSDK.h" AND MANUS_SDK_LIBRARY)
  file(READ "${MANUS_SDK_LIBRARY}" MANUS_SDK_LIBRARY_HEADER LIMIT 128)
  string(FIND "${MANUS_SDK_LIBRARY_HEADER}" "version https://git-lfs.github.com/spec/v1" MANUS_SDK_LFS_POINTER_OFFSET)
  if(MANUS_SDK_LFS_POINTER_OFFSET EQUAL -1)
    set(MANUS_SDK_AVAILABLE TRUE)
  else()
    set(MANUS_SDK_UNAVAILABLE_REASON
      "MANUS SDK library at ${MANUS_SDK_LIBRARY} is a Git LFS pointer, not the real shared library.")
  endif()
endif()

if(MANUS_SDK_AVAILABLE)
  find_package(Curses REQUIRED)

  add_executable(manus_data_publisher
    src/manus_data_publisher.cpp
    src/CalibrationFiles.cpp
    src/ClientPlatformSpecific.cpp
    src/ManusDataPublisher.cpp)
  target_include_directories(manus_data_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    "${MANUS_SDK_INCLUDE_DIR}")
  target_link_libraries(manus_data_publisher "${MANUS_SDK_LIBRARY}" ${CURSES_LIBRARIES})
  target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs ament_index_cpp)

  add_executable(manus_calibration_tool
    src/manus_calibration_tool.cpp
    src/CalibrationFiles.cpp)
  target_include_directories(manus_calibration_tool PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    "${MANUS_SDK_INCLUDE_DIR}")
  target_link_libraries(manus_calibration_tool "${MANUS_SDK_LIBRARY}" ${CURSES_LIBRARIES})
  target_compile_features(manus_calibration_tool PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(manus_calibration_tool ament_index_cpp)

  install(TARGETS manus_data_publisher manus_calibration_tool
    DESTINATION lib/${PROJECT_NAME})
  install(FILES "${MANUS_SDK_LIBRARY}" DESTINATION lib/${PROJECT_NAME})
  set_target_properties(manus_data_publisher manus_calibration_tool PROPERTIES INSTALL_RPATH "$ORIGIN")
else()
  message(WARNING
    "${MANUS_SDK_UNAVAILABLE_REASON} "
    "The manus_ros2 package will build without manus_data_publisher and "
    "manus_calibration_tool. Run scripts/install_manus_sdk.sh with "
    "MANUS_SDK_ARCHIVE, MANUS_SDK_DIR, or MANUS_SDK_URL, then rebuild to "
    "enable real MANUS glove input.")
endif()

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
