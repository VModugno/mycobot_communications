from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    
    use_realsense = LaunchConfiguration('use_realsense')
    use_realsense_launch_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='False'
    )
    
    cobot_node = Node(
            package='mycobot_interface_2',
            executable='cobot_comms',
            name='cobot_comms'
        )
    
    realsense_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            condition=IfCondition(
                PythonExpression([
                    use_realsense])
                )
            )

    return LaunchDescription([use_realsense_launch_arg, cobot_node, realsense_node])
