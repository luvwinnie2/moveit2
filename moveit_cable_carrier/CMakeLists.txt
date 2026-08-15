cmake_minimum_required(VERSION 3.16)
project(moveit_cable_carrier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(THIS_PACKAGE_DEPS
  moveit_core
  moveit_ros_planning
  geometric_shapes
  pluginlib
  rclcpp
  visualization_msgs
  geometry_msgs
  sensor_msgs
  urdf
  srdfdom
  std_msgs
  moveit_msgs
)

# ---------------------------------------------------------------------------
# core library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/rod_solver.cpp
  src/carrier_attacher.cpp
  src/carrier_registry.cpp
  src/collision_env_carrier.cpp
  src/trajectory_analysis.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${YAML_CPP_LIBRARIES})

# ---------------------------------------------------------------------------
# collision plugin
# ---------------------------------------------------------------------------
add_library(moveit_collision_detector_carrier_plugin SHARED
  src/collision_plugin_carrier.cpp
)
target_include_directories(moveit_collision_detector_carrier_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_collision_detector_carrier_plugin ${THIS_PACKAGE_DEPS})
target_link_libraries(moveit_collision_detector_carrier_plugin ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# tools
# ---------------------------------------------------------------------------
add_executable(carrier_benchmark src/tools/carrier_benchmark.cpp)
ament_target_dependencies(carrier_benchmark ${THIS_PACKAGE_DEPS})
target_link_libraries(carrier_benchmark ${PROJECT_NAME} Eigen3::Eigen)

add_executable(carrier_trajectory_check src/tools/carrier_trajectory_check.cpp)
ament_target_dependencies(carrier_trajectory_check ${THIS_PACKAGE_DEPS})
target_link_libraries(carrier_trajectory_check ${PROJECT_NAME} Eigen3::Eigen)

add_executable(carrier_visualizer src/tools/carrier_visualizer.cpp)
ament_target_dependencies(carrier_visualizer ${THIS_PACKAGE_DEPS})
target_link_libraries(carrier_visualizer ${PROJECT_NAME} Eigen3::Eigen)

# ---------------------------------------------------------------------------
# RViz panel
# ---------------------------------------------------------------------------
add_library(moveit_cable_carrier_rviz_panel SHARED
  src/rviz/carrier_panel.cpp
  include/moveit_cable_carrier/carrier_panel.hpp   # listed so AUTOMOC sees the Q_OBJECT
)
target_include_directories(moveit_cable_carrier_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_cable_carrier_rviz_panel
  rclcpp rviz_common pluginlib std_msgs)
target_link_libraries(moveit_cable_carrier_rviz_panel Qt5::Widgets)
target_compile_definitions(moveit_cable_carrier_rviz_panel PRIVATE QT_NO_KEYWORDS)

# ---------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME} moveit_collision_detector_carrier_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# The RViz panel is deliberately NOT in the export set. It is a pluginlib plugin, loaded at run
# time and linked by nobody -- but putting it in the export set makes its Qt5 and rviz_common link
# requirements part of this package's imported targets, so every downstream package that merely
# wants the carrier model is forced to find_package(Qt5) as well.
install(TARGETS moveit_cable_carrier_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS carrier_benchmark carrier_visualizer carrier_trajectory_check
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/compare_reference.py
  scripts/generate_reference_shapes.py
  scripts/make_carrier_urdf.py
  scripts/demo_trajectory_executor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.ja.md DESTINATION share/${PROJECT_NAME})
install(FILES
  collision_detector_carrier_plugin_description.xml
  rviz_panel_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(moveit_core collision_detector_carrier_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common rviz_panel_plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rod_solver test/test_rod_solver.cpp)
  if(TARGET test_rod_solver)
    target_link_libraries(test_rod_solver ${PROJECT_NAME} Eigen3::Eigen)
    ament_target_dependencies(test_rod_solver ${THIS_PACKAGE_DEPS})
  endif()

  ament_add_gtest(test_trajectory_analysis test/test_trajectory_analysis.cpp)
  if(TARGET test_trajectory_analysis)
    target_link_libraries(test_trajectory_analysis ${PROJECT_NAME} Eigen3::Eigen)
    ament_target_dependencies(test_trajectory_analysis ${THIS_PACKAGE_DEPS})
  endif()
endif()

ament_package()
