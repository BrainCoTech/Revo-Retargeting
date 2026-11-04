cmake_minimum_required(VERSION 3.16)
project(revo3_detect_ports LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PKG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(BRAINCO_REVO3_SDK_VENDOR_DIR "${PKG_ROOT}/vendor")
set(BRAINCO_REVO3_SDK_INCLUDE_DIR "${BRAINCO_REVO3_SDK_VENDOR_DIR}/dist/include")
set(BRAINCO_REVO3_SDK_LIB_DIR "${BRAINCO_REVO3_SDK_VENDOR_DIR}/dist/shared/linux")
set(BRAINCO_REVO3_SDK_LIB_PATH "${BRAINCO_REVO3_SDK_LIB_DIR}/libbc_revo3_sdk.so")

if(NOT EXISTS "${BRAINCO_REVO3_SDK_INCLUDE_DIR}/stark-sdk.h")
  message(FATAL_ERROR
    "stark-sdk.h not found. From revo3_driver run: bash scripts/download_sdk.sh")
endif()

if(NOT EXISTS "${BRAINCO_REVO3_SDK_LIB_PATH}")
  message(FATAL_ERROR
    "libbc_revo3_sdk.so not found. From revo3_driver run: bash scripts/download_sdk.sh")
endif()

add_library(bc_revo3_sdk SHARED IMPORTED GLOBAL)
set_target_properties(bc_revo3_sdk PROPERTIES
  IMPORTED_LOCATION "${BRAINCO_REVO3_SDK_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${BRAINCO_REVO3_SDK_INCLUDE_DIR}"
)

add_executable(detect_revo3_ports src/detect_revo3_ports.cpp)
target_include_directories(detect_revo3_ports PRIVATE "${BRAINCO_REVO3_SDK_INCLUDE_DIR}")
target_link_directories(detect_revo3_ports PRIVATE "${BRAINCO_REVO3_SDK_LIB_DIR}")
target_link_libraries(detect_revo3_ports PRIVATE bc_revo3_sdk)

set(_sdk_rpath "\$ORIGIN/../../vendor/dist/shared/linux")
set_target_properties(detect_revo3_ports PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
  BUILD_RPATH "${_sdk_rpath}"
  INSTALL_RPATH "${_sdk_rpath}"
  BUILD_WITH_INSTALL_RPATH TRUE
)
