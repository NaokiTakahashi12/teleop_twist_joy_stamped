#!/usr/bin/env -S python3

# MIT License
#
# Copyright (c) 2023 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_path = get_package_share_directory('teleop_twist_stamped_joy')
    teleop_twist_joy_pkg_share_path = get_package_share_directory('teleop_twist_joy')

    return [
        launch.actions.DeclareLaunchArgument(
            'joy_vel_topic',
            default_value='cmd_vel_stamped'
        ),
        launch.actions.DeclareLaunchArgument(
            'joy_topic',
            default_value='joy'
        ),
        launch.actions.DeclareLaunchArgument(
            'teleop_joy_config',
            default_value='ps3'
        ),
        launch.actions.DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0'
        ),
        launch.actions.DeclareLaunchArgument(
            'teleop_config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        teleop_twist_joy_pkg_share_path,
                        'config',
                        ''
                    )
                ),
                launch.substitutions.LaunchConfiguration('teleop_joy_config'),
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ]
        ),
        launch.actions.DeclareLaunchArgument(
            'joy_config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        this_pkg_share_path,
                        'config',
                        'joy_config.yaml'
                    )
                )
            ]
        )
    ]


# TODO Remove hard coded parameter
def generate_launch_nodes():
    return [
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                launch.substitutions.LaunchConfiguration('joy_config_filepath'),
                {'dev': launch.substitutions.LaunchConfiguration('joy_dev')}
            ],
            remappings={
                ('joy', launch.substitutions.LaunchConfiguration('joy_topic'))
            }
        ),
        launch_ros.actions.Node(
            package='teleop_twist_stamped_joy',
            executable='teleop_twist_stamped_joy_node',
            name='teleop_twist_joy_node',
            parameters=[
                launch.substitutions.LaunchConfiguration('teleop_config_filepath')
            ],
            remappings={
                ('~/cmd_vel_stamped', launch.substitutions.LaunchConfiguration('joy_vel_topic')),
                ('~/joy', launch.substitutions.LaunchConfiguration('joy_topic'))
            }
        )
    ]
