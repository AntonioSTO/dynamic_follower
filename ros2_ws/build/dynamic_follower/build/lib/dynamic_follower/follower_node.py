import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import transforms3d.euler

class SkeletonFollower(Node):
    def __init__(self):
        super().__init__('skeleton_follower')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(String, '/skeleton_pose', self.skeleton_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.last_pose = None  # Para armazenar última posição recebida

    def skeleton_callback(self, msg):
        x, y = map(float, msg.data.split())
        self.last_pose = (x, y)
        print(self.last_pose)

    def timer_callback(self):
        if self.last_pose is None:
            return  # Sem dados ainda

        x, y = self.last_pose
        yaw = 1.57

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Retorna (w, x, y, z)
        q = transforms3d.euler.euler2quat(0, 0, yaw)
        pose.pose.orientation.w = q[0]
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]

        self.publisher.publish(pose)
        self.get_logger().info(f'Published goal: x={x}, y={y}, yaw={yaw}')


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonFollower()
    rclpy.spin(node)
    rclpy.shutdown()
