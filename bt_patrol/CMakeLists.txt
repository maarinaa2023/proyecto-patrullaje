cmake_minimum_required(VERSION 3.5)
project(bt_patrol)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(patrol_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    rclcpp_action
    rosidl_default_generators
    rosidl_default_runtime  
    kobuki_ros_interfaces
    patrol_msgs
    std_msgs
    nav2_util
    geometry_msgs
    OpenCV
    cv_bridge
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(check_report_node SHARED src/bt_patrol/CheckReport.cpp)
add_library(everyone_here_node SHARED src/bt_patrol/IsEveryoneHere.cpp)
add_library(report_victim_node SHARED src/bt_patrol/ReportVictim.cpp)
add_library(reset_victim_node SHARED src/bt_patrol/ResetVictim.cpp)
add_library(get_waypoint_node SHARED src/bt_patrol/GetWaypoint.cpp)
add_library(move_node SHARED src/bt_patrol/Move.cpp)

list(APPEND plugin_libs
  check_report_node
  everyone_here_node
  report_victim_node
  reset_victim_node
  get_waypoint_node
  move_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${bt_plugin} "${cpp_typesupport_target}")
endforeach()

add_executable(patrol src/bt_patrol_main.cpp)
ament_target_dependencies(patrol ${dependencies})

add_executable(navigate src/nav_bt_main.cpp)
ament_target_dependencies(navigate ${dependencies})

install(TARGETS
  ${plugin_libs}
  patrol
  navigate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
include_directories(include)

install(DIRECTORY
  behaviour_tree_xml
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
