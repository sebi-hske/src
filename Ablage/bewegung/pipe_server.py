import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from bewegung.state_machine import ModeSelection
from std_msgs.msg import String
#implementierung action server?

class PipeServer(Node):
    def __init__(self):
        super().__init__('pipe_server')

        self.subscription_data = self.create_subscription(String, 'data', self.listener_data, 10)
        
    
    def listener_data(self, msg):
        data = msg.data
        data_tuple = tuple(map(float, data.split()))
        print(data_tuple)