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

find_package(zenoh_cpp_vendor QUIET)
find_package(zenohc QUIET CONFIG)

if(NOT TARGET zenohc::lib)
  find_package(PkgConfig QUIET)
endif()
if(NOT TARGET zenohc::lib AND PkgConfig_FOUND)
  pkg_check_modules(ZENOHC QUIET zenohc)
endif()

if(NOT TARGET zenohc::lib AND NOT ZENOHC_FOUND)
  find_path(
    ZENOHC_INCLUDE_DIR
    NAMES zenoh.h
    HINTS
      "/opt/ros/$ENV{ROS_DISTRO}/opt/zenoh_cpp_vendor/include"
      "$ENV{ZENOHC_ROOT}/include"
      "$ENV{ZENOH_C_ROOT}/include"
      "/usr/local/include"
      "/usr/include")

  find_library(
    ZENOHC_LIBRARY
    NAMES zenohc zenoh
    HINTS
      "/opt/ros/$ENV{ROS_DISTRO}/opt/zenoh_cpp_vendor/lib"
      "$ENV{ZENOHC_ROOT}/lib"
      "$ENV{ZENOH_C_ROOT}/lib"
      "/usr/local/lib"
      "/usr/lib")

  if(ZENOHC_INCLUDE_DIR AND ZENOHC_LIBRARY)
    set(ZENOHC_INCLUDE_DIRS "${ZENOHC_INCLUDE_DIR}")
    set(ZENOHC_LIBRARIES "${ZENOHC_LIBRARY}")
    set(ZENOHC_FOUND TRUE)
  endif()
endif()

if(NOT TARGET zenohc::lib AND NOT ZENOHC_FOUND)
  message(FATAL_ERROR
    "Could not find zenoh-c. Install zenoh_cpp_vendor/zenoh-c development files.")
endif()

add_executable(ue_zenoh_bridge src/bridge_node.cpp)
target_compile_features(ue_zenoh_bridge PUBLIC cxx_std_17)
if(TARGET zenohc::lib)
  target_link_libraries(ue_zenoh_bridge zenohc::lib)
else()
  target_include_directories(ue_zenoh_bridge PRIVATE ${ZENOHC_INCLUDE_DIRS})
  target_link_libraries(ue_zenoh_bridge ${ZENOHC_LIBRARIES})
endif()
ament_target_dependencies(ue_zenoh_bridge rclcpp rcutils sensor_msgs nav_msgs robots_dog_msgs)

install(TARGETS ue_zenoh_bridge
  DESTINATION lib/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
