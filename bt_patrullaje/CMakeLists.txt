cmake_minimum_required(VERSION 3.5)
project(bt_patrullaje)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    rclcpp_action
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(check_curr_section_node SHARED src/bt_patrol/checkCurrentSection.cpp)
add_library(check_others_node SHARED src/bt_patrol/checkOtherPositions.cpp)
add_library(check_victims_node SHARED src/bt_patrol/checkVictimReport.cpp)
add_library(get_victim_loc_node SHARED src/bt_patrol/getVictimLocation.cpp)
add_library(everyone_here_node SHARED src/bt_patrol/isEveryoneHere.cpp)
add_library(look_victim_node SHARED src/bt_patrol/look4Victim.cpp)
add_library(move_in_section_node SHARED src/bt_patrol/moveInSection.cpp)
add_library(nav2_assigned_section_node SHARED src/bt_patrol/nav2AssignedSection.cpp)
add_library(nav2_victim_location_node SHARED src/bt_patrol/nav2VictimLocation.cpp)
add_library(report_victim_node SHARED src/bt_patrol/reportVictim.cpp)
add_library(reset_victims_node SHARED src/bt_patrol/resetVictims.cpp)
add_library(synchronization_node SHARED src/bt_patrol/synchronization.cpp)

list(APPEND plugin_libs
  check_curr_section_node
  check_others_node
  check_victims_node
  get_victim_loc_node
  everyone_here_node
  look_victim_node
  move_in_section_node
  nav2_assigned_section_node
  nav2_victim_location_node
  report_victim_node
  reset_victims_node
  synchronization_node
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

install(DIRECTORY
  behaviour_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YourMessage.msg"
  # Add other message files if needed
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})


ament_package()
