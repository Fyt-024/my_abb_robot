#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher_node')
        
        # 话题名称必须与 controllers.yaml 中定义的一致
        topic_name = '/joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # 设置定时器，1秒后发送指令
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent_goal = False
        self.get_logger().info(f'准备向 {topic_name} 发送轨迹...')

    def timer_callback(self):
        if self.sent_goal:
            return
            
        msg = JointTrajectory()
        # 关节名称必须与 URDF 中的名称完全一致
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # --- 定义轨迹点 ---
        
        # 点 1: 初始位置 (全 0) - 2秒到达
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=2)

        # 点 2: 运动到某个姿态 - 5秒到达
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, -0.5, 1.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=5)

        # 点 3: 运动到另一个姿态 - 8秒到达
        point3 = JointTrajectoryPoint()
        point3.positions = [-0.5, 0.2, 0.0, -1.0, 0.5, 1.0]
        point3.time_from_start = Duration(sec=8)

        msg.points = [point1, point2, point3]

        self.publisher_.publish(msg)
        self.get_logger().info('轨迹指令已发送！机器人应该开始运动。')
        self.sent_goal = True

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

