cmake_minimum_required(VERSION 3.8)
project(ue_zenoh_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robots_dog_msgs REQUIRED)

# Prefer the Zenoh package provided by the active ROS 2 installation.  The
# vendor package prepends its private prefix to CMAKE_PREFIX_PATH, after which
# zenohc can be consumed exactly like a regular system CMake package.
find_package(zenoh_cpp_vendor QUIET)
find_package(zenohc CONFIG QUIET)

set(UE_ZENOHC_TARGET "")
foreach(candidate zenohc::lib zenohc::shared zenohc::zenohc zenohc)
  if(TARGET ${candidate})
    set(UE_ZENOHC_TARGET ${candidate})
    break()
  endif()
endforeach()

# Some distribution packages only install zenohc.pc.
if(NOT UE_ZENOHC_TARGET)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(SYSTEM_ZENOHC QUIET IMPORTED_TARGET zenohc)
    if(TARGET PkgConfig::SYSTEM_ZENOHC)
      set(UE_ZENOHC_TARGET PkgConfig::SYSTEM_ZENOHC)
    endif()
  endif()
endif()

# Finally support conventional /usr or user-supplied CMAKE_PREFIX_PATH roots
# even when the zenoh-c installation has no CMake or pkg-config metadata.
if(NOT UE_ZENOHC_TARGET)
  find_path(SYSTEM_ZENOHC_INCLUDE_DIR NAMES zenoh.h)
  find_library(SYSTEM_ZENOHC_LIBRARY NAMES zenohc)
  if(SYSTEM_ZENOHC_INCLUDE_DIR AND SYSTEM_ZENOHC_LIBRARY)
    add_library(ue_zenoh_bridge_zenohc UNKNOWN IMPORTED)
    set_target_properties(ue_zenoh_bridge_zenohc PROPERTIES
      IMPORTED_LOCATION "${SYSTEM_ZENOHC_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${SYSTEM_ZENOHC_INCLUDE_DIR}")
    set(UE_ZENOHC_TARGET ue_zenoh_bridge_zenohc)
  endif()
endif()

if(NOT UE_ZENOHC_TARGET)
  message(FATAL_ERROR
    "zenoh-c was not found. Install the ROS 2 zenoh_cpp_vendor package or a "
    "system zenoh-c development package, then source that environment or add "
    "its prefix to CMAKE_PREFIX_PATH.")
endif()

if(zenoh_cpp_vendor_FOUND)
  set(UE_ZENOHC_PROVIDER "ROS 2 zenoh_cpp_vendor")
else()
  set(UE_ZENOHC_PROVIDER "system zenoh-c")
endif()
message(STATUS "Using ${UE_ZENOHC_PROVIDER} via target ${UE_ZENOHC_TARGET}")

add_executable(ue_zenoh_bridge src/bridge_node.cpp)
target_compile_features(ue_zenoh_bridge PUBLIC cxx_std_17)
target_link_libraries(ue_zenoh_bridge ${UE_ZENOHC_TARGET} ZLIB::ZLIB)
ament_target_dependencies(ue_zenoh_bridge rclcpp rcutils sensor_msgs nav_msgs robots_dog_msgs)

install(TARGETS ue_zenoh_bridge
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/reliable_lidar_hz.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reliable_lidar_hz)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(pending_queue_test test/pending_queue_test.cpp)
  target_compile_features(pending_queue_test PRIVATE cxx_std_17)
  target_include_directories(pending_queue_test PRIVATE src)
  add_test(NAME pending_queue_test COMMAND pending_queue_test)
endif()

ament_package()
