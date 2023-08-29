#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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
"""Launch Webots Mavic 2 Pro driver."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('uav_launch')
    robot_description = pathlib.Path(
        os.path.join(package_dir, 'resource',
                     'mavic_webots.urdf')).read_text()

    mavic_driver = Node(package='webots_ros2_driver',
                        executable='driver',
                        output='screen',
                        additional_env={
                            'WEBOTS_CONTROLLER_URL':
                            controller_url_prefix() + 'Mavic_2_PRO'
                        },
                        parameters=[
                            {
                                'robot_description': robot_description
                            },
                        ])
    controller = Node(
        package='uav_control',
        # package='uav_control_cpp',
        executable='controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            },
        ])

    fusion = Node(
        # package='uav_fusion',
        package='uav_fusion_cpp',
        executable='fusion',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            },
        ])

    return [
        mavic_driver,
        controller,
        fusion,
    ]

class CloudRos2SupervisorLauncher(Node):
    def __init__(self, controller_url_prefix, output='screen', respawn=True, **kwargs):
        # Launch the Ros2Supervisor node
        super().__init__(
            package='webots_ros2_driver',
            executable='ros2_supervisor.py',
            output=output,
            # Set WEBOTS_HOME to the webots_ros2_driver installation folder
            # to load the correct libController libraries from the Python API
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix + 'Ros2Supervisor',
                            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            respawn=respawn,
            **kwargs
        )

def generate_launch_description():
    package_dir = get_package_share_directory('uav_launch')
    world = LaunchConfiguration('world')

    ros2_supervisor = CloudRos2SupervisorLauncher(controller_url_prefix="ws://172.17.0.1:2001")

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        ))

    return LaunchDescription([
        ros2_supervisor,

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())
