"""Bring up the chassis driver node together with robot_state_publisher.

The chassis model is selected by name from chassis_description/config/models.yaml:

    ros2 launch chassis_bringup bringup.launch.py model:=DD-M-HH

xacro_file and vehicle_param_file stay available to override either half of the
registry entry, e.g. to test a work-in-progress parameter set.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('chassis_description'), 'launch'))

from model_registry import default_model, model_entry  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    xacro_file = LaunchConfiguration('xacro_file').perform(context)
    vehicle_param_file = LaunchConfiguration('vehicle_param_file').perform(context)

    entry = model_entry(model)
    xacro_file = xacro_file or entry['xacro']
    vehicle_param_file = vehicle_param_file or entry['vehicle_param']

    xacro_path = os.path.join(
        get_package_share_directory('chassis_description'), 'urdf', xacro_file
    )
    vehicle_param_path = os.path.join(
        get_package_share_directory('chassis_bringup'), 'config', vehicle_param_file
    )

    for path in (xacro_path, vehicle_param_path):
        if not os.path.exists(path):
            raise RuntimeError(f"Model '{model}' refers to a missing file: {path}")

    robot_description = Command(['xacro ', xacro_path])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            output='screen',
            parameters=[vehicle_param_path],
        ),
    ]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model(),
        description='Chassis model name declared in chassis_description/config/models.yaml',
    )
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value='',
        description='Override the xacro file name under chassis_description/urdf/',
    )
    vehicle_param_file_arg = DeclareLaunchArgument(
        'vehicle_param_file',
        default_value='',
        description='Override the parameter yaml file name under chassis_bringup/config/',
    )

    return LaunchDescription([
        model_arg,
        xacro_file_arg,
        vehicle_param_file_arg,
        OpaqueFunction(function=_launch_setup),
    ])
