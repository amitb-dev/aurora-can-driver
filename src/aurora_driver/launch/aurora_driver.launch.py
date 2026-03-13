from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Launch file to start both the mock vehicle simulation 
    and the vehicle controller node simultaneously.
    """
    return LaunchDescription([
        # 1. Start the mock vehicle script using python3
        ExecuteProcess(
            cmd=['python3', 'src/mock_aurora_vehicle_node.py'],
            output='screen'
        ),

        # 2. Start your aurora_driver node
        Node(
            package='aurora_driver',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen'
        )
    ])