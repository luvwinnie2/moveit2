cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_tools)

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
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THIS_PACKAGE_DEPS
  moveit2_extended_core moveit2_extended_msgs behaviortree_cpp_v3 rclcpp pluginlib
  moveit_msgs geometry_msgs shape_msgs tf2
)

add_library(${PROJECT_NAME} SHARED src/tool_registry.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN OFF)

add_library(${PROJECT_NAME}_plugin SHARED src/tool_behaviors.cpp)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_plugin ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES CXX_VISIBILITY_PRESET default
                                                        VISIBILITY_INLINES_HIDDEN OFF)

add_executable(tool_manager src/tool_manager_node.cpp)
ament_target_dependencies(tool_manager ${THIS_PACKAGE_DEPS})
target_link_libraries(tool_manager ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS tool_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES behavior_plugin.yaml behavior_loader_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(moveit2_extended_core behavior_loader_plugin_description.xml)
ament_index_register_resource("moveit2_extended_behavior_plugin"
  CONTENT "share/${PROJECT_NAME}/behavior_plugin.yaml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tool_registry test/test_tool_registry.cpp)
  if(TARGET test_tool_registry)
    target_link_libraries(test_tool_registry ${PROJECT_NAME})
    ament_target_dependencies(test_tool_registry ${THIS_PACKAGE_DEPS})
    target_compile_definitions(test_tool_registry PRIVATE CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()
endif()

ament_package()
