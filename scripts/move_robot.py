#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher_node')
        
        topic_name = '/joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # 关键修改：等待订阅者连接
        self.get_logger().info(f'正在等待控制器连接到话题: {topic_name} ...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
            # 如果一直卡在这里，说明话题名字不对，或者仿真没启动
            
        self.get_logger().info('控制器已连接！准备发送指令...')
        
        # 延迟一小会儿确保连接稳定
        time.sleep(1.0)
        self.send_trajectory()

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # 点 1: 2秒后移动到位置 A
        point1 = JointTrajectoryPoint()
        point1.positions = [0.5, 0.5, -0.5, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=2)

        # 点 2: 5秒后移动到位置 B
        point2 = JointTrajectoryPoint()
        point2.positions = [-0.5, 0.0, 0.5, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=5)

        msg.points = [point1, point2]

        self.publisher_.publish(msg)
        self.get_logger().info('>>> 轨迹指令已发送！请观察 Gazebo <<<')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    # 发送完就退出，不需要 spin 循环
    # node.destroy_node() 
    # rclpy.shutdown()
    # 为了防止报错，我们简单地让它在这里结束
    pass

if __name__ == '__main__':
    main()

