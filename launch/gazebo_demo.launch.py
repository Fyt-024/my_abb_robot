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
    
    # 启动Gazebo（暂停状态）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false', 'pause': 'true'}.items()
    )
    
    # 在Gazebo中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'my_abb_irb1200',
                   '-z', '0.0'],  # 确保机器人在地面上
        output='screen'
    )
    
    # 取消暂停Gazebo
    unpause = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        unpause
    ])

