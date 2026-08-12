"""Spawn a chassis model in Gazebo Classic.

The model is selected by name from chassis_description/config/models.yaml:

    ros2 launch chassis_description gazebo.launch.py model:=DD-M-HH
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from model_registry import default_model, xacro_path  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)

    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_path(model)])

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', model, '-topic', 'robot_description'],
            output='screen',
        ),
    ]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model(),
        description='Chassis model name declared in chassis_description/config/models.yaml',
    )

    return LaunchDescription([
        model_arg,
        OpaqueFunction(function=_launch_setup),
    ])
