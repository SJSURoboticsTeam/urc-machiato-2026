"""Isaac Sim bridge launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    camera_namespace = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='Camera namespace'
    )
    
    use_single_camera = DeclareLaunchArgument(
        'use_single_camera',
        default_value='true',
        description='Use single camera mode (Isaac Sim default topics)'
    )
    
    bridge_node = Node(
        package='autonomy_core',
        executable='bridge_node',
        name='isaac_sim_bridge',
        parameters=[{
            'use_sim_time': True,
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'use_single_camera': LaunchConfiguration('use_single_camera'),
        }],
        output='screen'
    )
    
    sim_safety_adapter = Node(
        package='autonomy_core',
        executable='sim_safety_adapter',
        name='sim_safety_adapter',
        parameters=[{
            'simulation_mode': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        camera_namespace,
        use_single_camera,
        bridge_node,
        sim_safety_adapter,
    ])
