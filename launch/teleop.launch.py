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
