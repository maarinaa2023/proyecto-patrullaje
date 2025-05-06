#!/bin/sh
export ROS_DOMAIN_ID=69
export ROS_DISCOVERY_SERVER="10.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix dds_server)/share/dds_server/config/superClientConfig.xml"

# Stop the daemon (only if it's running)
  if ros2 daemon status > /dev/null 2>&1; then
    ros2 daemon stop
    
    # Small delay to ensure clean shutdown
    sleep 1
  fi
  
  # Start the daemon with new settings
  ros2 daemon start