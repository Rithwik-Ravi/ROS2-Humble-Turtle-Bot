import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    urdf_path = os.path.join(package_dir, 'urdf', 'rover.urdf')

    robot_description_config = xacro.process_file(urdf_path)
    urdf_content = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # --- This points to your new wide_maze.world ---
    world_path = os.path.join(package_dir, 'worlds', 'wide_maze.world')
    # -----------------------------------------------

    # --- ADDED: New, correct Gazebo launch (FIXED SYNTAX) ---
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_path,
        }.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "dolly"])
    
    # --- ADDED: 5-second delay for spawner ---
    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        gazebo_node,
        
        delayed_spawn_entity,
        
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            output='screen',
            remappings=[('image', '/camera/depth/image_raw'),
                        ('camera_info', '/camera/depth/camera_info'),
                        ('scan', '/scan')],
            parameters=[{
                'range_max': 1.5,
                'use_sim_time': use_sim_time
            }]
        )
    ])