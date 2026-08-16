cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_mujoco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(moveit2_extended_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# MuJoCo ships as a Python wheel here -- there is no system package and no CMake config -- but the
# wheel contains both the shared library and the full C headers. Ask Python where it put them
# rather than hard-coding a site-packages path that changes with every interpreter version.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_DIR OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE MUJOCO_FOUND)
if(NOT MUJOCO_FOUND EQUAL 0)
  message(FATAL_ERROR "mujoco is not importable. pip install mujoco")
endif()
file(GLOB MUJOCO_LIB "${MUJOCO_DIR}/libmujoco.so*")
list(GET MUJOCO_LIB 0 MUJOCO_LIB)
if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "no mujoco.h under ${MUJOCO_DIR}/include")
endif()
message(STATUS "MuJoCo: ${MUJOCO_LIB}")

set(THIS_PACKAGE_DEPS
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
  moveit2_extended_msgs moveit_msgs sensor_msgs std_srvs)

add_library(${PROJECT_NAME} SHARED src/mujoco_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_DIR}/include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME} ${MUJOCO_LIB})
# The library lives inside a Python package, so the loader needs to be told where to find it at
# run time; without this the plugin fails to dlopen with "libmujoco.so.3: cannot open".
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${MUJOCO_DIR}" BUILD_WITH_INSTALL_RPATH TRUE
  CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN OFF)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/urdf_to_mujoco.py scripts/mujoco_viewer.py
        DESTINATION lib/${PROJECT_NAME})
install(FILES mujoco_system_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
