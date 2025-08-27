import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleControlNode(Node):
    def __init__(self):
        super().__init__('turtle_control_node')

        # Inicializa variáveis de estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.pose_received = False
        self.goal_received = False

        # Ganhos de controle
        self.k_linear = 1.0
        self.k_angular = 6.0

        # Subscreve à pose atual da tartaruga
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Subscreve ao objetivo (goal) que queremos seguir
        self.create_subscription(Pose, '/turtle1/pose_goal', self.goal_callback, 10)

        # Publica comandos de velocidade
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Timer de controle
        self.timer = self.create_timer(0.1, self.pub_callback)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pose_received = True
        self.get_logger().info(f'[POSE] x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.goal_received = True
        self.get_logger().info(f'[GOAL] x={self.x_goal:.2f}, y={self.y_goal:.2f}')

    def pub_callback(self):
        self.get_logger().info("Executando pub_callback...")

        if not self.pose_received:
            self.get_logger().warn("Pose ainda não recebida.")
            return
        if not self.goal_received:
            self.get_logger().warn("Goal ainda não recebido.")
            return

        # Calcula erro de posição
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normaliza

        # Gera comandos
        linear = self.k_linear * distance
        angular = self.k_angular * angle_error

        # Limita velocidades
        linear = min(linear, 2.0)
        angular = max(min(angular, 2.0), -2.0)

        # Para ao chegar no objetivo
        if distance < 0.1:
            linear = 0.0
            angular = 0.0
            self.get_logger().info('[STATUS] Objetivo alcançado!')

        # Publica comando
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher_.publish(cmd)

        self.get_logger().info(f'[CMD] linear={linear:.2f}, angular={angular:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
