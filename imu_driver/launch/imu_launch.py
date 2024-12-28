import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'port', 
        default_value='/dev/tty/USB0',  
        description='Path to the IMU serial port'
    )
    
    imu_node = Node(
        package='imu_driver',  
        namespace='imu',
        executable='driver',   
        name='imu_driver',
        parameters=[{
            'port': LaunchConfiguration('port')
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        imu_node
    ])
