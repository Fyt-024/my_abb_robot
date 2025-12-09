#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class GazeboJointBridge(Node):
    def __init__(self):
        super().__init__('gazebo_joint_bridge')
        
        # 订阅joint_states话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        # 为每个关节创建发布者
        self.joint_pubs = {}
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        for joint in joint_names:
            topic = f'/model/my_abb_irb1200/joint/{joint}/0/cmd_pos'
            self.joint_pubs[joint] = self.create_publisher(Float64MultiArray, topic, 10)
        
        self.get_logger().info('Gazebo Joint Bridge started')
        
    def joint_states_callback(self, msg):
        # 将joint_states转发到Gazebo
        for i, name in enumerate(msg.name):
            if name in self.joint_pubs and i < len(msg.position):
                cmd = Float64MultiArray()
                cmd.data = [msg.position[i]]
                self.joint_pubs[name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboJointBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

