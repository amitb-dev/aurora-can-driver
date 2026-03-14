from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define parameters for command-line configuration
    throttle_arg = DeclareLaunchArgument('drive_throttle', default_value='-300')
    duration_arg = DeclareLaunchArgument('drive_duration_sec', default_value='5.0')

    return LaunchDescription([
        throttle_arg,
        duration_arg,

        # Vehicle simulation node
        Node(
            package='aurora_driver',
            executable='mock_vehicle',
            name='mock_aurora_vehicle'
        ),

        # Main vehicle controller node
        Node(
            package='aurora_driver',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen',
            # Map launch configurations to ROS 2 parameters
            parameters=[{
                'drive_throttle': LaunchConfiguration('drive_throttle'),
                'drive_duration_sec': LaunchConfiguration('drive_duration_sec'),
            }]
        )
    ])