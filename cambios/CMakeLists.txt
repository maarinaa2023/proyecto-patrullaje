cmake_minimum_required(VERSION 3.5)
project(nav_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    rclcpp_action
    geometry_msgs
    OpenCV
    cv_bridge
    std_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(getwp_bt_node SHARED src/nav_bt/GetWaypoint.cpp)
add_library(move_bt_node SHARED src/nav_bt/Move.cpp)
add_library(checkzone_bt_node SHARED src/nav_bt/CheckZone.cpp)
add_library(rescue_bt_node SHARED src/nav_bt/RescueWaypoint.cpp)

list(APPEND plugin_libs
  getwp_bt_node
  move_bt_node
  checkzone_bt_node
  rescue_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nav_bt_main src/nav_bt_main.cpp)
ament_target_dependencies(nav_bt_main ${dependencies})

install(TARGETS
  ${plugin_libs}
  nav_bt_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config behavior_tree_xml
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
