#!/usr/bin python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='shm2ros',
            executable='shm2ros',
            name='shm2ros',
            output='screen',
            emulate_tty=True,
            parameters=[
                '/root/ros2_ws/src/shm2ros/config/shm2ros_params.yaml',
                {
                    # 'shm_read_st': 0.01,
                    # 'twist_pub_st': 0.01,
                    # 'twist_topic': 'cmd_vel',
                    # 'same_msg_limit': 3.,
                }
            ]
        ),
    ])

    return ld