cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_gpu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit2_extended_core REQUIRED)
find_package(moveit2_extended_behaviors REQUIRED)
find_package(moveit2_extended_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
  moveit2_extended_core moveit2_extended_behaviors moveit2_extended_msgs
  behaviortree_cpp_v3 rclcpp pluginlib moveit_msgs geometry_msgs sensor_msgs
)

# No CUDA, no cuRobo, no torch anywhere in this target. These are service clients: they build and
# register on a machine with no GPU at all, which is what lets one Objective XML run on the
# workstation and on the robot.
add_library(${PROJECT_NAME}_plugin SHARED src/gpu_behaviors.cpp)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_plugin ${THIS_PACKAGE_DEPS})
# Default visibility, deliberately: ROS message types travel between this .so and the Objective
# server's through the blackboard, and hidden visibility gives each library its own typeinfo, so
# the any_cast fails at run time with a message that says nothing.
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES CXX_VISIBILITY_PRESET default
                                                        VISIBILITY_INLINES_HIDDEN OFF)

install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/curobo_service.py DESTINATION lib/${PROJECT_NAME})
install(FILES behavior_plugin.yaml behavior_loader_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(moveit2_extended_core behavior_loader_plugin_description.xml)
ament_index_register_resource("moveit2_extended_behavior_plugin"
  CONTENT "share/${PROJECT_NAME}/behavior_plugin.yaml")

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
