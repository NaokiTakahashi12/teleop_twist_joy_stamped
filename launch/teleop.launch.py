#!/usr/bin/env -S python3

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
    teleop_twist_joy_pkg_path = get_package_share_directory('teleop_twist_joy')

    return [
        launch.actions.DeclareLaunchArgument(
            'joy_vel',
            default_value='cmd_vel_stamped'
        ),
        launch.actions.DeclareLaunchArgument(
            'joy',
            default_value='joy'
        ),
        launch.actions.DeclareLaunchArgument(
            'joy_config',
            default_value='ps3'
        ),
        launch.actions.DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0'
        ),
        launch.actions.DeclareLaunchArgument(
            'config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        teleop_twist_joy_pkg_path,
                        'config',
                        ''
                    )
                ),
                launch.substitutions.LaunchConfiguration('joy_config'),
                launch.substitutions.TextSubstitution(text='.config.yaml')
        ])
    ]

# TODO Remove hard coded parameter

def generate_launch_nodes():
    return [
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': launch.substitutions.LaunchConfiguration('joy_dev'),
                'deadzone': 0.3,
                'autorepeat_rate': 20.0
            }]
        ),
        launch_ros.actions.Node(
            package='teleop_twist_stamped_joy',
            executable='teleop_twist_stamped_joy_node',
            name='teleop_twist_joy_node',
            parameters=[
                launch.substitutions.LaunchConfiguration('config_filepath')
            ],
            remappings={
                ('~/cmd_vel_stamped', launch.substitutions.LaunchConfiguration('joy_vel')),
                ('~/joy', launch.substitutions.LaunchConfiguration('joy'))
            }
        )
    ]
