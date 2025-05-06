#!/bin/bash

# 1. Set ROS_DOMAIN_ID to create a virtual network for your robot fleet
# Each robot fleet should use a different domain ID (0-232)
export ROS_DOMAIN_ID=69
export ROS_DISCOVERY_SERVER="10.0.0.1:11811"
#fastdds discovery -i 69 -l 10.0.0.1 -p 11811

#export ROS2_EASY_MODE=10.0.0.1
#Server needs to be started as fastdds discovery start -d 69 10.0.0.1:69

# 2. Configure network quality of service
# For reliable communication in multi-robot systems, use explicit QoS settings
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastrtps_profile.xml
