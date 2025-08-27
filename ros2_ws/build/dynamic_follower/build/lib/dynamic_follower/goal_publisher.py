# goal_publisher.py

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import random

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Pose, '/turtle1/pose_goal', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self,msg):
        pub = Pose()
        pub.x,pub.y = float(msg.data.split)
        pub.theta = 0.0
        self.publisher_.publish(pub)
        self.get_logger().info(f'Publicando nova pose: x={pub.x:.2f}, y={pub.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
