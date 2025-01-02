import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from ar_interface.action import Mode
from ar_pipe_server.state_machine import ModeSelection
from std_msgs.msg import String
#implementierung action server?

class PipeServer(Node):
    def __init__(self):
        super().__init__('pipe_server')

        self.subscription_data = self.create_subscription(String, 'data', self.listener_data, 10)
        self.cmd_move = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            Mode,       #action server und goals hier ändern
            'mode',
            execute_callback=self.execute_callback,
            goal_callback= self.goal_callback,
            handle_accepted_callback= self.handle_accepted_callback,
            cancel_callback= self.cancel_callback)

        self.callback_group = ReentrantCallbackGroup()
        self.goal_lock = threading.Lock()
        self.mode_selection = ModeSelection()
            

    def listener_data(self, msg):
        data = msg.data
        self.data_tuple = tuple(map(float, data.split()))
        #print(self.data_tuple)

    def goal_callback(self, goal_request):
        self.get_logger().info('Recieved goal to start driving with velocity: ')

        PLACEHOLDER = None
        if PLACEHOLDER is not None:
            self.get_logger().info('Goal was rejected because of PLACEHOLDER')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling goal, stopping robot')

        self.cmd_move.linear.x = 0.0
        self.cmd_move.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_move)

        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('starting to drive with velocity: ')

        while rclpy.ok():

            print(self.data_tuple)
            
            (id, offset, distance) = self.data_tuple

            print(int(id))
            print(offset)
            print(distance)

            mode = int(id)

            self.mode_selection._state = mode
            #hier aufrufen der state machine


            time.sleep(0.1)

        return self.determince_action_result(goal_handle)
    
    def determine_action_result(self, goal_handle):
        result = Mode.Result()

        if goal_handle.is_active:
            self.get_logger().info('Driving succeded')
            goal_handle.succeed()
            result.reached = True
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Driving goal was canceled')
        else:
            if goal_handle.is_active():
                goal_handle.abort()
            self.get_logger().info('Driving goal was aborted')
        return result
    
def main():
    print('Pipe Server active')
    rclpy.init()
    try:
        pipe_server = PipeServer()
        mt_executer = MultiThreadedExecutor()
        rclpy.spin(pipe_server, executor=mt_executer)
        pipe_server.destroy()
    finally:
        rclpy.shutdown()