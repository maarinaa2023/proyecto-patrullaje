## Proyecto Patrullaje
# Multi-Robot Patrolling and Security/Rescue System

This project implements a scalable multi-robot system for autonomous patrolling and security/rescue operations, with distributed decision-making and coordination capabilities through behavior trees and DDS communication.

## Overview

The system enables multiple robots to independently patrol designated areas while maintaining the ability to collaborate when emergencies are detected. The core functionality includes:

- Distributed communication between all robots in the network
- Autonomous patrolling of assigned areas
- Human detection using YOLO object detection
- Coordinated response when a person is detected
- Return to patrol after assistance is provided

## System Architecture

### Communication Layer

The system uses Fast DDS (Data Distribution Service) for scalable, real-time communication between robots. This implementation provides:

- **Reliable Communication**: Guaranteed message delivery across robots
- **Scalability**: Support for multiple robots without performance degradation
- **Network Flexibility**: Works across robots connected to the same WiFi network

### Behavior Control

Robot behavior is managed through Behavior Trees that enable complex decision-making:

- **Patrol Mode**: Default behavior where robots monitor their assigned areas
- **Rescue Mode**: Triggered when a person is detected, redirecting robots to provide assistance
- **Return-to-Patrol**: Automatic resumption of patrol duties after rescue operations

### Object Detection

Person detection is implemented using YOLO (You Only Look Once) models:

- **Real-time Detection**: Fast inference suitable for robot operation
- **Flexible Configuration**: Supports different camera setups across the robot fleet
- **Robust Detection**: Works across various lighting conditions and environments

## Components

### 1. dds_server

A centralized DDS server that facilitates communication between all robots in the system.

- **Purpose**: Enables scalable pub/sub communication between robots
- **Implementation**: Uses Fast DDS server for efficient message passing
- **Requirements**: All robots must be connected to the same WiFi network
- **Alternatives Explored**: Cloud-based server (rejected due to latency/throughput issues)

### 2. bt_patrol

Implements the behavior tree logic for robot patrol operations.

- **Features**:
  - Condition checking for task switching
  - Message handling for coordination
  - State management for robot operations

### 3. nav_bt

Navigation-specific behavior tree implementation.

- **Features**:
  - Path planning for patrol routes
  - Dynamic path adjustment
  - Waypoint management
  - Response to rescue triggers

### 4. yoloddx

YOLO-based detection system customized for multi-robot operation.

- **Features**:
  - Namespace management for multiple robots
  - Camera configuration adaptation
  - Real-time person detection
  - Alert generation on positive detection

## Behavior Tree Structure

The system implements a behavior tree architecture as shown in the referenced Groot visualization. Key nodes include:

- **Root**: Top-level control node
- **Fallback**: Selects between patrolling and rescue operations
- **ReactiveSequence**: Monitors for rescue triggers during patrol
- **Sequence**: Orders operations during patrol and rescue
- **Move**: Navigation to target locations
- **RescueWaypoint**: Determines rescue destination
- **RetryUntilSuccessful**: Ensures completion of critical tasks
- **IsEveryoneHere**: Checks if all required robots have arrived
- **CheckVictim**: Verifies the presence of a person in need
- **Scan4Report**: Gathers environmental data

## Setup Requirements

### Hardware Requirements

- Multiple robots with:
  - ROS2-compatible control system
  - Cameras (configurations may vary)
  - Network connectivity
  - Sufficient computing power for YOLO inference

### Software Requirements

- ROS2 (tested on Jazzy)
- Fast DDS
- Behavior Tree libraries
- YOLO dependencies (specific to your implementation)
- Navigation stack components (nav2)

### Robot Configuration

For each robot in your fleet:

1. Set unique namespace and TF prefixes
2. Configure camera parameters in the YOLO launch files
3. Define patrol areas and waypoints
4. Ensure proper network connectivity

## Troubleshooting

### Common Issues

1. **Communication Failures**:
   - Ensure all robots are on the same WiFi network
   - Verify DDS server is running and accessible
   - Check ROS_DOMAIN_ID is consistent across all robots

2. **Detection Issues**:
   - Verify camera calibration and configuration
   - Check YOLO model loading correctly
   - Ensure sufficient lighting for reliable detection

3. **Navigation Problems**:
   - Confirm map availability
   - Check localization accuracy
   - Verify waypoint coordinates

## Future Work

- Cloud-based communication with improved reliability (current cloud implementation isn't good enough)
- Enhanced coordination algorithms
- Multi-person rescue prioritization
- Integration with additional sensor modalities

## Contributing

Contributions to improve the system are welcome. Please follow the standard fork-and-pull-request workflow.


### Hecho por Marina, Irene, Matias y Javier<3
