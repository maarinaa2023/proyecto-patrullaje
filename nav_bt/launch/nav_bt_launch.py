# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_bt')
    param_file = os.path.join(pkg_dir, 'config', 'zones_sim.yaml')

    nav = Node(
        package='nav_bt',
        executable='nav_bt_main',
        name='nav_bt',
        output='screen',
        remappings=[
            ('/output_vel', '/cmd_vel'),
            ('/input_scan', '/scan')
        ],
        parameters=[
            param_file
        ])

    ld = LaunchDescription()
    ld.add_action(nav)
    return ld
