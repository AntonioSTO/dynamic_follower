import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import pyttsx3
import logging

class collectMsg(Node):
    def __init__(self):
        super().__init__('is_audio_node')
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            force=True,
        )

        self.init_variables()
        self.init_subscribers()
        self.init_publishers()

        self.locations = [
            (-2.0, -4.0, 0.0), #porta
            (1.5, -2.0, 0.0), #servidor
            (0.0, 0.0, 0.0), #origem
            (2.0, 2.0, 0.0), #armario
            (-2.0, 0.0, 0.0) #computador random
        ]

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.goal_received = False
        self.pose_received = False
        self.k_linear = 1.0  # Ganho para controle linear
        self.k_angular = 6.0  # Ganho para controle angular

    def init_publishers(self):
        self.publisher_ = self.create_publisher(
            String,
            'teste',
            10
            )

    def init_subscribers(self):
        self.create_subscription(
            String,
            'voice_msg',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        if "porta" in msg.data:
            print(msg.data)
            location = self.locations[0]
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
            logging.info(f"Command sent by operator to {location}")
            self.send_goal(location)

        elif "servidor" in msg.data:
            location = self.locations[1]
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
            logging.info(f"Command sent by operator to {location}")
            self.send_goal(location)

        elif "origem" in msg.data:
            location = self.locations[2]
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
            logging.info(f"Command sent by operator to {location}")
            self.send_goal(location)
        
        elif "armario" in msg.data:
            location = self.locations[3]
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
            logging.info(f"Command sent by operator to {location}")
            self.send_goal(location)
    
        elif "computador" in msg.data:
            location = self.locations[4]
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
            logging.info(f"Command sent by operator to {location}")
            self.send_goal(location)
        
        elif "Operador não autorizado." in msg.data:
            mensagem = String()
            mensagem.data = msg.data
            self.publisher_.publish(mensagem)
        
        else:
            logging.info(f"Command not recognized")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def send_goal(self, location):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = location[0]
        goal_msg.pose.pose.position.y = location[1]
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, location[2])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    print('Aguandando comando...')
    rclpy.init(args=args)
    node = collectMsg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()