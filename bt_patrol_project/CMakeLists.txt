cmake_minimum_required(VERSION 3.5)
project(bt_patrol_project)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(patrol_msgs REQUIRED)


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
)

add_library(check_report_node SHARED src/bt_patrol/CheckReport.cpp)
add_library(everyone_here_node SHARED src/bt_patrol/IsEveryoneHere.cpp)
add_library(report_victim_node SHARED src/bt_patrol/ReportVictim.cpp)
add_library(reset_victim_node SHARED src/bt_patrol/ResetVictim.cpp)

list(APPEND plugin_libs
  check_report_node
  everyone_here_node
  report_victim_node
  reset_victim_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrol src/bt_patrol_main.cpp)
ament_target_dependencies(patrol ${dependencies})

install(TARGETS
  ${plugin_libs}
  patrol
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
