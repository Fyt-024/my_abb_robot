#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math

class FlatCircleDrawer(Node):
    def __init__(self):
        super().__init__('flat_circle_drawer')
        
        topic_name = '/joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        self.get_logger().info('等待控制器连接...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.5)
            
        self.get_logger().info('控制器已连接！开始计算平面圆形轨迹...')
        time.sleep(1.0)
        self.send_trajectory()

    def solve_ik(self, x, y, z):
        """
        简化的逆运动学求解器 (针对 IRB 1200 构型)
        输入: 目标坐标 (x, y, z)
        输出: 关节角度 [j1, j2, j3, j4, j5, j6]
        """
        # 机器人几何参数 (来自您的 URDF)
        L_BASE = 0.399   # J2 高度
        L_ARM1 = 0.448   # 大臂长
        L_ARM2 = 0.533   # 小臂 + 法兰长 (0.451 + 0.082)

        # 1. 计算 Joint 1 (底座旋转)
        # 简单的极坐标转换，让机器人转向目标方向
        j1 = math.atan2(y, x)

        # 2. 计算 Joint 2 和 Joint 3 (平面 2连杆 IK)
        # 目标在机器人垂直平面内的投影距离 (水平距离)
        r_horizontal = math.sqrt(x**2 + y**2)
        # 目标相对于 J2 的高度
        z_local = z - L_BASE
        
        # 目标点到 J2 的直线距离
        distance = math.sqrt(r_horizontal**2 + z_local**2)

        # 安全检查：如果目标太远，就限制在最大半径内
        if distance > (L_ARM1 + L_ARM2):
            self.get_logger().warn('目标点超出工作空间！')
            return None

        # 利用余弦定理求解三角形
        # alpha 是大臂与 "J2-目标连线" 的夹角
        # beta 是 J2-目标连线 与 水平线 的夹角
        cos_angle_3 = (distance**2 - L_ARM1**2 - L_ARM2**2) / (2 * L_ARM1 * L_ARM2)
        # 限制数值范围防止报错
        cos_angle_3 = max(min(cos_angle_3, 1.0), -1.0)
        
        # j3_internal 是肘部弯曲的内角
        j3_internal = math.acos(cos_angle_3)
        # 换算成 URDF 的 joint_3 角度 (通常 0 度是直臂，向下弯是正/负)
        # 对于 IRB 1200，joint_3 向下弯曲通常为负
        j3 = -(math.pi - j3_internal)

        # 计算 j2
        beta = math.atan2(z_local, r_horizontal)
        alpha = math.acos((L_ARM1**2 + distance**2 - L_ARM2**2) / (2 * L_ARM1 * distance))
        j2 = (math.pi / 2) - (beta + alpha) # 调整坐标系，视具体零位定义而定

        # 3. 计算 Joint 5 (手腕俯仰)
        # 我们希望末端垂直向下 (Pitch = -90度 或 -pi/2)
        # 全局 Pitch = j2 + j3 + j5
        # 所以 j5 = -pi/2 - j2 - j3
        j5 = 0.0 - j2 - j3 

        return [j1, j2, j3, 0.0, j5, 0.0]

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # --- 圆形参数定义 ---
        center_x = 0.5      # 圆心在机器人前方 0.5米处
        center_y = 0.0      # 正前方
        center_z = 0.4      # 高度 0.4米 (大约在腰部高度)
        radius = 0.20       # 半径 15厘米
        
        num_points = 100    # 轨迹点数
        total_time = 10.0   # 总时间

        for i in range(num_points + 1):
            # 1. 生成圆上的坐标点
            angle = 2 * math.pi * (i / num_points)
            target_x = center_x + radius * math.cos(angle) # 前后变化
            target_y = center_y + radius * math.sin(angle) # 左右变化
            target_z = center_z                            # 高度不变

            # 2. 逆运动学求解
            joints = self.solve_ik(target_x, target_y, target_z)
            
            if joints:
                point = JointTrajectoryPoint()
                point.positions = joints
                
                # 时间计算
                time_now = total_time * (i / num_points)
                sec = int(time_now)
                nanosec = int((time_now - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                
                msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('>>> 平面圆形轨迹已发送！请观察 RViz 中的红色轨迹 <<<')

def main(args=None):
    rclpy.init(args=args)
    node = FlatCircleDrawer()
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

