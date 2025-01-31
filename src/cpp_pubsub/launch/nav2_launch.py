#!/usr/bin/env python3
"""
nav2_launch.py

A simple launch file for Navigation2 using your custom configuration.
It brings up the Nav2 stack using the nav2_bringup launch file.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory for our package
    nav2_setup_dir = get_package_share_directory('cpp_pubsub')
    # Get the directory for the nav2_bringup package (must be installed)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to the parameters file
    params_file = os.path.join(nav2_setup_dir, 'config', 'nav2_params.yaml')

    # Declare if we're using simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include the Nav2 bringup launch file, passing our parameters file and use_sim_time flag
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        nav2_bringup_launch,
    ])
