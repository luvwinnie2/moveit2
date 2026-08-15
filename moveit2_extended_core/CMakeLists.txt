cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# BehaviorTree.CPP v3 and v4 both live in namespace BT with incompatible layouts. Nav2 is built
# against v3, so a process that links both corrupts memory instead of failing to link. Catch an
# accidental v4 dependency at configure time rather than at 3am.
if(DEFINED behaviortree_cpp_DIR)
  message(WARNING
    "BehaviorTree.CPP v4 (behaviortree_cpp) appears to be on the prefix path. This project builds "
    "against v3 (behaviortree_cpp_v3) because Nav2 does. Linking both into one process is an ODR "
    "violation that shows up as heap corruption, not as a link error.")
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit2_extended_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

set(THIS_PACKAGE_DEPS
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  pluginlib
  ament_index_cpp
  moveit2_extended_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  geometry_msgs
  rcl_interfaces
)

# ---------------------------------------------------------------------------
# core library: context, bases, loader seam, objective library, server
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/path_utils.cpp
  src/blackboard_text.cpp
  src/behavior_parameters.cpp
  src/behavior_context.cpp
  src/async_behavior_base.cpp
  src/tree_introspection.cpp
  src/status_logger.cpp
  src/objective_library.cpp
  src/objective_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} tinyxml2::tinyxml2)
# Behaviors registered by one shared library are ticked by another and exchange typed values
# through BT::Any, which compares std::type_index across library boundaries. Hidden visibility
# would give each library its own type_info and every cross-library port read would throw.
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN OFF)

# ---------------------------------------------------------------------------
# built-in behaviors, in their own library so the core does not depend on them
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_behaviors SHARED
  src/behaviors/log_message.cpp
  src/behaviors/wait_for_duration.cpp
  src/behaviors/check_blackboard_value.cpp
  src/behaviors/core_behaviors_loader.cpp
)
target_include_directories(${PROJECT_NAME}_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_behaviors ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME}_behaviors ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_behaviors PROPERTIES CXX_VISIBILITY_PRESET default
                                                           VISIBILITY_INLINES_HIDDEN OFF)

# ---------------------------------------------------------------------------
# server executable
# ---------------------------------------------------------------------------
add_executable(objective_server src/objective_server_main.cpp)
ament_target_dependencies(objective_server ${THIS_PACKAGE_DEPS})
target_link_libraries(objective_server ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_behaviors
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS objective_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch objectives DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/integration_objective_server.py DESTINATION share/${PROJECT_NAME}/test)
install(FILES behavior_plugin.yaml behavior_loader_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(${PROJECT_NAME} behavior_loader_plugin_description.xml)

# THIS is the line every Behavior package copies. It is what lets the Objective server find a
# package's behavior_plugin.yaml without anyone editing a list in a different package.
ament_index_register_resource("moveit2_extended_behavior_plugin"
  CONTENT "share/${PROJECT_NAME}/behavior_plugin.yaml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_behavior_parameters test/test_behavior_parameters.cpp)
  if(TARGET test_behavior_parameters)
    target_link_libraries(test_behavior_parameters ${PROJECT_NAME})
    ament_target_dependencies(test_behavior_parameters ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_blackboard_text test/test_blackboard_text.cpp)
  if(TARGET test_blackboard_text)
    target_link_libraries(test_blackboard_text ${PROJECT_NAME})
    ament_target_dependencies(test_blackboard_text ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_tree_introspection test/test_tree_introspection.cpp)
  if(TARGET test_tree_introspection)
    target_link_libraries(test_tree_introspection ${PROJECT_NAME})
    ament_target_dependencies(test_tree_introspection ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_objective_library test/test_objective_library.cpp)
  if(TARGET test_objective_library)
    target_link_libraries(test_objective_library ${PROJECT_NAME})
    ament_target_dependencies(test_objective_library ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_behavior_registration test/test_behavior_registration.cpp)
  if(TARGET test_behavior_registration)
    target_link_libraries(test_behavior_registration ${PROJECT_NAME} ${PROJECT_NAME}_behaviors)
    ament_target_dependencies(test_behavior_registration ${THIS_PACKAGE_DEPS})
  endif()

  # Guards the bt_compat.hpp firewall: a v3-only spelling outside that header would make the v4
  # migration a search-and-replace across every Behavior instead of one file.
  ament_add_gtest(test_bt_compat_firewall test/test_bt_compat_firewall.cpp)
  if(TARGET test_bt_compat_firewall)
    target_compile_definitions(test_bt_compat_firewall PRIVATE
      SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endif()

ament_package()
