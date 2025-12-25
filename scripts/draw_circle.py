#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer_node')
        
        topic_name = '/joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        self.get_logger().info('等待控制器连接...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
            
        self.get_logger().info('控制器已连接！准备画圆...')
        time.sleep(1.0)
        self.send_circle_trajectory()

    def send_circle_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # --- 参数设置 ---
        num_points = 100        # 轨迹点的数量（点越多越平滑）
        cycle_time = 10.0       # 画一圈需要的总时间（秒）
        
        # 中心姿态（让机器人在前方抬起手）
        center_j2 = 0.5         # 大臂中心角度
        center_j3 = -0.5        # 小臂中心角度
        
        # 半径（摆动的幅度，单位是弧度）
        radius = 0.3            

        # --- 生成轨迹点 ---
        for i in range(num_points + 1):
            point = JointTrajectoryPoint()
            
            # 计算当前进度 (0 到 2*pi)
            angle = 2 * math.pi * (i / num_points)
            
            # 利用三角函数生成圆形运动
            # joint_2 负责上下，joint_3 负责前后
            # 这种组合会在机器人的侧面垂直平面画一个圆
            val_j2 = center_j2 + radius * math.sin(angle)
            val_j3 = center_j3 + radius * math.cos(angle)
            
            # 设置关节角度
            # joint_1 保持 0 (朝前) 或 1.57 (朝左)
            # joint_4, 5, 6 保持固定
            point.positions = [0.0, val_j2, val_j3, 0.0, 0.0, 0.0]
            
            # 计算该点的时间戳
            time_now = cycle_time * (i / num_points)
            sec = int(time_now)
            nanosec = int((time_now - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送包含 {num_points} 个点的圆形轨迹指令！')

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    time.sleep(1) # 等待日志打印
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

