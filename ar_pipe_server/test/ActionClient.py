import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ar_interface.action import Mode

class TestActionClient:
    

    def send_goal(self):
        self.action_client = ActionClient(self, Mode, 'velocity')
        #goal_msg = Mode.Goal()
        #goal_msg.velocity = order

        self.action_client.wait_for_server()

        self.action_client.send_goal(0.1)

 
