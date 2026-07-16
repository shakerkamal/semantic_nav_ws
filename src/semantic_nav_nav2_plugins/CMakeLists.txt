cmake_minimum_required(VERSION 3.8)
project(semantic_nav_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(semantic_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rtabmap_msgs QUIET)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/validate_semantic.cpp
  src/escalate_to_llm_recovery.cpp
  src/path_clear_condition.cpp
  src/capture_blockage_context.cpp
  src/query_semantic_context.cpp
  src/register_nodes.cpp
  src/emit_obstacle_signal.cpp
  src/operator_prompt.cpp
  src/compute_standoff_pose.cpp
  src/has_responsible_object_candidate.cpp
  src/wait_for_barrier_clear.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  semantic_nav_interfaces
  geometry_msgs
  builtin_interfaces
  nav_msgs
  nav2_msgs
  std_msgs
  nav2_util
  tf2_ros
)

if(rtabmap_msgs_FOUND)
  ament_target_dependencies(${library_name} rtabmap_msgs)
  message(STATUS
    "semantic_nav_nav2_plugins: CleanupLocalGrids headers will be used "
    "when provided by the installed rtabmap_msgs package")
else()
  message(WARNING
    "semantic_nav_nav2_plugins: rtabmap_msgs not found; "
    "WaitForBarrierClear will skip cached-grid cleanup")
endif()

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bt_plugins test/test_bt_plugins.cpp)
  target_include_directories(test_bt_plugins PRIVATE include)
  target_link_libraries(test_bt_plugins ${library_name})
  ament_target_dependencies(test_bt_plugins
    rclcpp
    nav2_behavior_tree
    behaviortree_cpp_v3
    semantic_nav_interfaces
    geometry_msgs
    builtin_interfaces
    nav_msgs
    nav2_msgs
    std_msgs
    nav2_util
    tf2_ros
  )
  if(rtabmap_msgs_FOUND)
    ament_target_dependencies(test_bt_plugins rtabmap_msgs)
  endif()

  ament_add_gtest(test_bt_plugins_live test/test_bt_plugins_live.cpp)
  target_include_directories(test_bt_plugins_live PRIVATE include)
  target_link_libraries(test_bt_plugins_live ${library_name})
  ament_target_dependencies(test_bt_plugins_live
    rclcpp
    nav2_behavior_tree
    behaviortree_cpp_v3
    semantic_nav_interfaces
    geometry_msgs
    builtin_interfaces
    nav_msgs
    nav2_msgs
    std_msgs
    nav2_util
    tf2_ros
  )
  if(rtabmap_msgs_FOUND)
    ament_target_dependencies(test_bt_plugins_live rtabmap_msgs)
  endif()

  ament_add_gtest(
    test_wait_for_barrier_clear
    test/test_wait_for_barrier_clear.cpp
  )
  target_include_directories(test_wait_for_barrier_clear PRIVATE include)
  target_link_libraries(test_wait_for_barrier_clear ${library_name})
  ament_target_dependencies(test_wait_for_barrier_clear
    rclcpp
    nav2_behavior_tree
    behaviortree_cpp_v3
    geometry_msgs
    nav_msgs
    nav2_msgs
  )
  if(rtabmap_msgs_FOUND)
    ament_target_dependencies(test_wait_for_barrier_clear rtabmap_msgs)
  endif()
endif()

ament_package()
