import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
    
    # Robot State Publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # 在Gazebo中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_abb_irb1200'],
        output='screen'
    )
    
    # Joint State Publisher (用于发布关节状态)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_entity
    ])
