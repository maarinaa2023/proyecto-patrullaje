cmake_minimum_required(VERSION 3.8)
project(patrol_msgs)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Add other dependencies as needed

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolMsgs.msg"
  # Add other message files
  DEPENDENCIES geometry_msgs
)

ament_package()
