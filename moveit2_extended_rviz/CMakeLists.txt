cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit2_extended_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# AUTOMOC has to be on before the target is declared, and the header carrying Q_OBJECT has to be
# listed as a source below or moc never sees it and the link fails on the vtable.
set(CMAKE_AUTOMOC ON)

set(THIS_PACKAGE_DEPS
  moveit2_extended_msgs
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  rcl_interfaces
)

add_library(moveit2_extended_rviz_panel SHARED
  src/objectives_panel.cpp
  include/moveit2_extended_rviz/objectives_panel.hpp
)
target_include_directories(moveit2_extended_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit2_extended_rviz_panel ${THIS_PACKAGE_DEPS})
target_link_libraries(moveit2_extended_rviz_panel Qt5::Widgets)
# Qt's `signals`, `slots` and `emit` are plain macros that collide with anything else using those
# words -- Boost and some ROS headers do. Q_SIGNALS/Q_SLOTS are used instead.
target_compile_definitions(moveit2_extended_rviz_panel PRIVATE QT_NO_KEYWORDS)

# Installed WITHOUT ament_export_targets on purpose. Nothing links this -- RViz dlopens it -- and
# putting it in the export set would make its Qt5 and rviz_common link interface part of this
# package's public contract, forcing every downstream consumer to find_package(Qt5). That exact
# mistake in moveit_cable_carrier broke the carrier Behaviors package's build.
install(TARGETS moveit2_extended_rviz_panel
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rviz_panel_plugin_description.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common rviz_panel_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # A compile-only guard. The panel's real failure mode is a threading mistake, which no unit test
  # catches, so what is tested here is the one thing that IS mechanically checkable: that no ROS
  # callback in the panel touches a Qt widget or blocks on a future.
  ament_add_gtest(test_panel_thread_rules test/test_panel_thread_rules.cpp)
  if(TARGET test_panel_thread_rules)
    target_compile_definitions(test_panel_thread_rules PRIVATE
      PANEL_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/src/objectives_panel.cpp")
  endif()
endif()

ament_package()
