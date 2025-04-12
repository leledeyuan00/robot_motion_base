cmake_minimum_required(VERSION 3.8)
project(robot_motion_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROBOT_MOTION_BASE robot_motion_base)
set(ROBOT_MOTION_BASE_VERSION 0.1.0)
set(ROBOT_MOTION_BASE_DESCRIPTION "Robot motion base library")

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  geometry_msgs
  std_srvs
  tf2_eigen
  tf2_kdl
  orocos_kdl
  Eigen3
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Library for motion base
include_directories(include/robot_motion_base)
set(HEADER_FILES include/robot_motion_base/robot_motion_base.hpp)
add_library(${ROBOT_MOTION_BASE} SHARED
  src/robot_motion_base.cpp
  ${HEADER_FILES})
ament_target_dependencies(${ROBOT_MOTION_BASE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${ROBOT_MOTION_BASE} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${ROBOT_MOTION_BASE} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)

INSTALL(
  TARGETS ${ROBOT_MOTION_BASE}
  EXPORT export_${ROBOT_MOTION_BASE}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${ROBOT_MOTION_BASE})

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
