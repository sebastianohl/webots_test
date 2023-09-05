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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix


class CloudWebotsController(ExecuteProcess):

    def __init__(self,
                 ip_address,
                 port,
                 output='screen',
                 respawn=False,
                 remappings=[],
                 namespace='',
                 parameters=[],
                 robot_name='',
                 **kwargs):
        webots_controller = (os.path.join(
            get_package_share_directory('webots_ros2_driver'), 'scripts',
            'webots-controller'))

        protocol = "tcp"

        robot_name_option = [] if not robot_name else [
            '--robot-name=' + robot_name
        ]
        ip_address_option = [] if not ip_address else [
            '--ip-address=' + ip_address
        ]

        ros_arguments = []
        for item in remappings:
            key, value = item
            remap = f'{key}:={value}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        if (namespace):
            remap = f'__ns:=/{namespace}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    parameter = [
                        f'{key}:=', value if isinstance(value, Substitution)
                        else TextSubstitution(text=str(value))
                    ]
                    ros_arguments.append('-p')
                    ros_arguments.append(parameter)
        file_parameters = [
            item for item in parameters if isinstance(item, str)
        ]

        ros_args = ['--ros-args'] if ros_arguments else []
        params_file = ['--params-file'] if file_parameters else []

        node_name = 'webots_controller' + (
            ('_' + robot_name) if robot_name else '')
        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                *robot_name_option,
                ['--protocol=', protocol],
                *ip_address_option,
                ['--port=', port],
                'ros2',
                *ros_args,
                *ros_arguments,
                *params_file,
                *file_parameters,
            ],
            name=node_name,
            respawn=respawn,
            # Set WEBOTS_HOME to package directory to load correct controller library
            additional_env={
                'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')
            },
            **kwargs)

    def execute(self, context: LaunchContext):
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)


def get_ros2_nodes(ip_address, port, *args):
    package_dir = get_package_share_directory('uav_launch')
    #robot_description = str(
    #    pathlib.Path(os.path.join(package_dir, 'resource',
    #                              'mavic_webots.urdf')))

    #mavic_driver = Node(package='webots_ros2_driver',
    #                    executable='driver',
    #                    output='screen',
    #                    additional_env={
    #                        'WEBOTS_CONTROLLER_URL':
    #                        controller_url_prefix + 'Mavic_2_PRO'
    #                    },
    #                    parameters=[
    #                        {
    #                            'robot_description': robot_description
    #                        },
    #                    ])
    robot_description_path = os.path.join(package_dir, 'resource',
                                          'mavic_webots.urdf')
    mavic_driver = CloudWebotsController(robot_name='Mavic_2_PRO',
                                         ip_address=ip_address,
                                         port=port,
                                         parameters=[
                                             {
                                                 'robot_description':
                                                 robot_description_path
                                             },
                                         ],
                                         respawn=True)

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

    def __init__(self,
                 controller_url_prefix,
                 output='screen',
                 respawn=True,
                 **kwargs):
        # Launch the Ros2Supervisor node
        super().__init__(
            package='webots_ros2_driver',
            executable='ros2_supervisor.py',
            output=output,
            # Set WEBOTS_HOME to the webots_ros2_driver installation folder
            # to load the correct libController libraries from the Python API
            additional_env={
                'WEBOTS_CONTROLLER_URL':
                controller_url_prefix + 'Ros2Supervisor',
                'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')
            },
            respawn=respawn,
            **kwargs)


def generate_launch_description():
    package_dir = get_package_share_directory('uav_launch')
    world = LaunchConfiguration('world')

    ip_address = "webots-cloud.ii.svc.cluster.local"
    port = os.getenv("WEBOTS_PORT")

    print(f"tcp://{ip_address}:{port}")
    ros2_supervisor = CloudRos2SupervisorLauncher(
        controller_url_prefix=f"tcp://{ip_address}:{port}/")

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
    ] + get_ros2_nodes(ip_address=ip_address, port=port))
