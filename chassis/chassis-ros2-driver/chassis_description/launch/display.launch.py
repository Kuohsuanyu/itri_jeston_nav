"""Display a chassis model in RViz2 with joint_state_publisher_gui.

The model is selected by name from chassis_description/config/models.yaml:

    ros2 launch chassis_description display.launch.py model:=DD-M-HH
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from model_registry import default_model, rviz_path, xacro_path  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)

    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_path(model)])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='static_view',
            arguments=['-d', rviz_path(model)],
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
