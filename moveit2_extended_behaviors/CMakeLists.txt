cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit2_extended_core REQUIRED)
find_package(moveit2_extended_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_DEPS
  moveit2_extended_core
  moveit2_extended_msgs
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  shape_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/port_conversions.cpp
  src/moveit_goal_helpers.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
# See the note in moveit2_extended_core: BT::Any compares std::type_index across shared-library
# boundaries, so hidden visibility would make every cross-library port read throw.
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN OFF)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_port_conversions test/test_port_conversions.cpp)
  if(TARGET test_port_conversions)
    target_link_libraries(test_port_conversions ${PROJECT_NAME})
    ament_target_dependencies(test_port_conversions ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_moveit_goal_helpers test/test_moveit_goal_helpers.cpp)
  if(TARGET test_moveit_goal_helpers)
    target_link_libraries(test_moveit_goal_helpers ${PROJECT_NAME})
    ament_target_dependencies(test_moveit_goal_helpers ${THIS_PACKAGE_DEPS})
  endif()
endif()

ament_package()
