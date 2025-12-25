#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')
        
        # 创建 Marker 发布者，话题为 /end_effector_path
        self.marker_pub = self.create_publisher(Marker, '/end_effector_path', 10)
        
        # 初始化 TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 初始化 Marker 线条属性
        self.marker = Marker()
        self.marker.header.frame_id = "base_link" # 轨迹基于基座坐标系
        self.marker.type = Marker.LINE_STRIP      # 类型：线条
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.01  # 线条粗细 1cm
        self.marker.color.a = 1.0   # 不透明度
        self.marker.color.r = 1.0   # 红色
        self.marker.pose.orientation.w = 1.0
        
        # 定时器：0.1秒记录一次
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('轨迹可视化节点已启动...')

    def timer_callback(self):
        try:
            # 获取 link_6 (末端) 相对于 base_link (基座) 的位置
            # 如果您的末端不叫 link_6，请修改这里
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                'link_6', 
                rclpy.time.Time())
            
            p = Point()
            p.x = t.transform.translation.x
            p.y = t.transform.translation.y
            p.z = t.transform.translation.z
            
            # 只有位置变动超过 1mm 才记录，避免静止时点重叠
            if not self.marker.points or \
               (abs(p.x - self.marker.points[-1].x) > 0.001 or 
                abs(p.y - self.marker.points[-1].y) > 0.001 or 
                abs(p.z - self.marker.points[-1].z) > 0.001):
                
                self.marker.points.append(p)
                self.marker.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub.publish(self.marker)
                
        except Exception:
            pass

def main():
    rclpy.init()
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

