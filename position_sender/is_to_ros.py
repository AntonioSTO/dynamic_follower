from ros_pb2 import ROSMessage
from google.protobuf.struct_pb2 import Struct
from is_wire.core import Channel, Message


class SkeletonPosition:
    def __init__(self, channel, topic):
        self._channel = Channel(channel)
        self.topic = topic

    def get_ros_message(self, msg):
        ros_message = ROSMessage()
        ros_message.type = "std_msgs/msg/Float32MultiArray"
        
        msg_dict = {
        'layout': {
            'dim': [],          
            'data_offset': 0   
        },
        'data': msg      
        }

        struct_msg = Struct()
        struct_msg.update(msg_dict)
        
        ros_message = ROSMessage(content=struct_msg)
        ros_message.type = "std_msgs/msg/Float32MultiArray"

        return ros_message

    def send_to(self, msg):
        print("Positions to send:", msg)
        ros_message = self.get_ros_message(msg)
        message = Message(content=ros_message)
        self._channel.publish(message, topic=self.topic)
        print(f"Published on {self.topic}: {msg}")
