import os
from launch import LaunchDescription
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
    
    # Joint State Publisher GUI节点
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RVIZ2节点
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])

