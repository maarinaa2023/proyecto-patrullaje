cmake_minimum_required(VERSION 3.8)
project(proyecto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  yaml-cpp
)

include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/proyecto/KobukiPatroller.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Compilar el nodo de detección del target
add_executable(kobuki_patroller_main src/kobuki_patroller_main.cpp)
ament_target_dependencies(kobuki_patroller_main ${dependencies})
target_link_libraries(kobuki_patroller_main ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  kobuki_patroller_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
