cmake_minimum_required(VERSION 3.8)
project(ue_zenoh_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robots_dog_msgs REQUIRED)

set(ZENOHC_VERSION "1.9.0")
set(ZENOHC_VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/zenoh-c")
set(ZENOHC_VENDOR_INCLUDE_DIR "${ZENOHC_VENDOR_ROOT}/include")

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR
    "The bundled zenoh-c ${ZENOHC_VERSION} library currently supports Linux x86_64 only.")
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" UE_ZENOH_SYSTEM_PROCESSOR)
if(NOT UE_ZENOH_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  message(FATAL_ERROR
    "The bundled zenoh-c ${ZENOHC_VERSION} library does not support "
    "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}.")
endif()

set(
  ZENOHC_VENDOR_LIBRARY
  "${ZENOHC_VENDOR_ROOT}/lib/linux-x86_64/libzenohc.so")

foreach(required_file
    "${ZENOHC_VENDOR_INCLUDE_DIR}/zenoh.h"
    "${ZENOHC_VENDOR_LIBRARY}")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR
      "Missing bundled zenoh-c ${ZENOHC_VERSION} file: ${required_file}")
  endif()
endforeach()

add_library(zenohc_vendor SHARED IMPORTED GLOBAL)
set_target_properties(zenohc_vendor PROPERTIES
  IMPORTED_LOCATION "${ZENOHC_VENDOR_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ZENOHC_VENDOR_INCLUDE_DIR}")

add_executable(ue_zenoh_bridge src/bridge_node.cpp)
target_compile_features(ue_zenoh_bridge PUBLIC cxx_std_17)
target_link_libraries(ue_zenoh_bridge zenohc_vendor)
ament_target_dependencies(ue_zenoh_bridge rclcpp rcutils sensor_msgs nav_msgs robots_dog_msgs)
set_target_properties(ue_zenoh_bridge PROPERTIES
  BUILD_RPATH "${ZENOHC_VENDOR_ROOT}/lib/linux-x86_64"
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
# Emit DT_RPATH instead of DT_RUNPATH so a ROS environment containing another
# libzenohc.so in LD_LIBRARY_PATH cannot override the bundled 1.9.0 runtime.
set_property(
  TARGET ue_zenoh_bridge APPEND_STRING
  PROPERTY LINK_FLAGS " -Wl,--disable-new-dtags")

install(TARGETS ue_zenoh_bridge
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/reliable_lidar_hz.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reliable_lidar_hz)

install(FILES "${ZENOHC_VENDOR_LIBRARY}"
  DESTINATION lib/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

install(FILES
    ThirdParty/zenoh-c/README.md
    ThirdParty/zenoh-c/LICENSE
    ThirdParty/zenoh-c/NOTICE.md
  DESTINATION share/${PROJECT_NAME}/third_party/zenoh-c)

ament_package()
