#!/bin/bash
export ROS_DOMAIN_ID=69
export ROS_DISCOVERY_SERVER="10.0.0.1:11811"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/../config/superClientConfig.xml"

# Resolve it to an absolute path
CONFIG_FILE="$(readlink -f "$CONFIG_FILE")"

export FASTRTPS_DEFAULT_PROFILES_FILE="$CONFIG_FILE"

# Stop the daemon (only if it's running)
  if ros2 daemon status > /dev/null 2>&1; then
    ros2 daemon stop
    
    # Small delay to ensure clean shutdown
    sleep 1
  fi
  
  # Start the daemon with new settings
  ros2 daemon start
