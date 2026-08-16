cmake_minimum_required(VERSION 3.16)
project(moveit2_extended_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorStatus.msg"
  "msg/BehaviorTreeLog.msg"
  "msg/ObjectiveState.msg"
  "msg/TreeNodeInfo.msg"
  "msg/TreeStructure.msg"
  "msg/PortInfo.msg"
  "msg/BehaviorInfo.msg"
  "msg/ObjectiveInfo.msg"
  "msg/UserPrompt.msg"
  "msg/UserResponse.msg"

  "msg/CarrierDiagnostics.msg"
  "msg/CarrierWaypointResult.msg"
  "msg/CarrierSegmentFatigue.msg"
  "msg/CarrierTrajectoryReport.msg"

  "msg/ArmWaypoint.msg"
  "msg/ToolInfo.msg"

  "srv/ListObjectives.srv"
  "srv/GetObjective.srv"
  "srv/ListBehaviors.srv"
  "srv/ReloadObjectives.srv"
  "srv/ValidateObjectiveXml.srv"
  "srv/SaveObjectiveXml.srv"

  "srv/SaveWaypoint.srv"
  "srv/ListWaypoints.srv"
  "srv/GetWaypoint.srv"
  "srv/DeleteWaypoint.srv"
  "srv/RenameWaypoint.srv"

  "srv/SwitchTool.srv"
  "srv/ListTools.srv"
  "srv/GetActiveTool.srv"

  "srv/SetSimulationState.srv"
  "srv/SolveIKBatch.srv"
  "srv/PlanWithGpu.srv"

  "action/ExecuteObjective.action"

  DEPENDENCIES builtin_interfaces action_msgs rcl_interfaces std_msgs sensor_msgs geometry_msgs
               moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
