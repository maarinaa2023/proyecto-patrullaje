# DDS Server

## Overview

This project implements a DDS (Data Distribution Service) server for managing communication in a multi-robot system. It includes scripts for setting up the client and launching the server.

## Prerequisites

- ROS 2 (Robot Operating System 2)
- Fast DDS (Fast Data Distribution Service)

## Usage

### Client Setup

To set up the client, you need to either source the package or run the setup script:

- **Source the package:**
  ```bash
  source install/setup.bash
  ```
  or

- **Source the setup script:**
  ```bash
  source shell_scripts/setupClient.sh
  ```

### Server Launch

To launch the server, you have two options:

- **Using ROS 2 launch:**
  ```bash
  ros2 launch serverlaunch.py
  ```
    or
- **Running the discovery server script:**
  ```bash
  source shell_scripts/startDiscoveryServer.sh
  ```

## Configuration

The configuration for the client is defined in `config/superClientConfig.xml`. You can modify this file to adjust the DDS settings as needed.

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or issues, please contact the maintainer:

- **Name:** javi<3
- **Email:** javi.rm2005@gmail.com