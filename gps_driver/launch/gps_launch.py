from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for GPS puck'
        ),
        Node(
            package='gps_driver',
            executable='driver',
            name='gps_driver_node',
            output='screen',
            arguments=[port],
        ),
    ])
