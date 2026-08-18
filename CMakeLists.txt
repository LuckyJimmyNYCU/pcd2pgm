cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# ---------------------------------------------------------------------------
# Vendored CSF (Cloth Simulation Filter), Apache-2.0.
# Source: https://github.com/jianboqi/CSF, see third_party/CSF/README.md
#
# Built as a separate static library so that upstream's warnings are not mixed
# in with this package's, and so it can be swapped for a system install later
# without touching the node.
# ---------------------------------------------------------------------------
set(CSF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/CSF)

add_library(csf STATIC
  ${CSF_DIR}/CSF.cpp
  ${CSF_DIR}/Cloth.cpp
  ${CSF_DIR}/Constraint.cpp
  ${CSF_DIR}/Particle.cpp
  ${CSF_DIR}/Rasterization.cpp
  ${CSF_DIR}/XYZReader.cpp
  ${CSF_DIR}/c2cdist.cpp
  ${CSF_DIR}/point_cloud.cpp
)

# SYSTEM: keeps this package's -Wpedantic from firing on upstream headers
# (point_cloud.h uses an anonymous struct).
target_include_directories(csf SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CSF_DIR}>
)

set_target_properties(csf PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Upstream code: don't apply this package's warning flags to it.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(csf PRIVATE -w)
endif()

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
add_executable(pcd2pgm src/pcd2pgm.cpp)

target_include_directories(pcd2pgm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pcd2pgm
  csf
  ${PCL_LIBRARIES}
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# pcl_conversions is header-only in Jazzy; pull its include dirs in.
ament_target_dependencies(pcd2pgm pcl_conversions)

target_compile_definitions(pcd2pgm PRIVATE ${PCL_DEFINITIONS})

install(TARGETS pcd2pgm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()