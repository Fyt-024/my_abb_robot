import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('my_abb_robot')
    
    # 处理URDF文件
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_irb1200.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_abb_irb1200'],
        output='screen'
    )
    
    # 延迟加载控制器（等待Gazebo和机器人完全加载）
    delayed_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "joint_trajectory_controller"],
                output="screen",
            )
        ]
    )
    
    # Joint State Publisher GUI (用于测试控制)
    joint_state_gui = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rqt_joint_trajectory_controller',
                executable='rqt_joint_trajectory_controller',
                name='rqt_joint_trajectory_controller',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delayed_controller_spawner,
        joint_state_gui,
    ])

