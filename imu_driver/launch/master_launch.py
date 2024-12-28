from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the port arguments for GPS and IMU
    gps_port_arg = DeclareLaunchArgument(
        'gps_port', 
        default_value='/dev/tty/USB0',  
        description='Path to the GPS serial port'
    )
    
    imu_port_arg = DeclareLaunchArgument(
        'imu_port', 
        default_value='/dev/tty/USB1',  
        description='Path to the IMU serial port'
    )

    # Get the launch file paths for GPS and IMU drivers
    gps_launch_file = os.path.join(get_package_share_directory('gps_driver'), 'launch', 'gps_launch.py')
    imu_launch_file = os.path.join(get_package_share_directory('imu_driver'), 'launch', 'imu_launch.py')

    return LaunchDescription([
        gps_port_arg,
        imu_port_arg,

        # Include the GPS driver launch file and pass the gps_port argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch_file),
            launch_arguments={'port': LaunchConfiguration('gps_port')}.items()
        ),
        
        # Include the IMU driver launch file and pass the imu_port argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_file),
            launch_arguments={'port': LaunchConfiguration('imu_port')}.items()
        ),
    ])

