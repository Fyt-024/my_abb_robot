#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        
    def timer_callback(self):
        msg = Float64MultiArray()
        # 创建一个简单的正弦波运动
        t = self.counter * 0.1
        msg.data = [
            0.0,  # joint_1
            math.sin(t) * 0.5,  # joint_2 - 正弦运动
            math.cos(t) * 0.3,  # joint_3
            0.0,  # joint_4
            math.sin(t * 2) * 0.2,  # joint_5
            0.0   # joint_6
        ]
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    print("开始移动机器人...")
    print("按Ctrl+C停止")
    try:
        rclpy.spin(robot_mover)
    except KeyboardInterrupt:
        pass
    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

