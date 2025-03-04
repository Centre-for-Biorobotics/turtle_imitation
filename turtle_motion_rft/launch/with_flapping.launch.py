#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_path = get_package_share_directory('turtle_motion_rft')
model_path = package_path + '/ml_models/imitation_model.keras'

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtle_motion_rft',  
            executable='stroke_imitated.py',     
            name='stroke_imitated',
            parameters=[
                {'model_path': model_path}
            ]
        ),
        Node(
            package='turtle_motion_rft',  
            executable='flapping_smc.py',     
            name='flapping_smc',
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()