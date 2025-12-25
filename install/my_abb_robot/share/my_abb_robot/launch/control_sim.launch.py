import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # 1. 获取包路径
    pkg_path = get_package_share_directory('my_abb_robot')
    
    # 2. 处理URDF文件
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_irb1200.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # 3. Robot State Publisher (发布 TF 坐标变换，必须保留)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # 4. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # 5. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_abb_irb1200'],
        output='screen'
    )

    # ====================================================
    # 以下是与原文件的主要区别：加载控制器
    # ====================================================

    # 加载关节状态广播器 (负责发布关节当前真实位置)
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 加载轨迹控制器 (负责接收运动指令)
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # 定义启动顺序：
    # 1. 机器人生成(spawn)完成后 -> 加载状态广播器(jsb)
    # 2. 状态广播器(jsb)加载完成后 -> 加载轨迹控制器(jtc)
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_jtc],
            )
        ),
    ])

