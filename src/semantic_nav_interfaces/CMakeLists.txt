cmake_minimum_required(VERSION 3.8)
project(semantic_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticDB.msg"
  "msg/RecoveryTrigger.msg"
  "msg/ObjectInstance.msg"

  "srv/ResolveLocation.srv"
  "srv/ValidatePose.srv"
  "srv/ParseSemanticCommand.srv"
  "srv/ProposeRecovery.srv"
  "srv/RequestRecovery.srv"
  "srv/RefreshLocalObjects.srv"
  "srv/MatchResponsibleObject.srv"
  "srv/OperatorDecision.srv"
  "srv/NavigateToQuery.srv"

  "action/ExecutePose.action"

  DEPENDENCIES geometry_msgs builtin_interfaces action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()