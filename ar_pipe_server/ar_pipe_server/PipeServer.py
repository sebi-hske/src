import sys
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy.wait_for_message
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
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)  
        
        self.cmd_move = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            Mode,       #action server und goals hier ändern
            'velocity',
            execute_callback=self.execute_callback,
            goal_callback= self.goal_callback,
            handle_accepted_callback= self.handle_accepted_callback,
            cancel_callback= self.cancel_callback)
        
        self.callback_group = ReentrantCallbackGroup()
        self.goal_lock = threading.Lock()
        self.mode_selection = ModeSelection()
        #ArucoDistance()
            

    def listener_data(self, msg):
        data = msg.data
        self.data_tuple = tuple(map(float, data.split()))
        #print(self.data_tuple)

    def odom_callback(self,msg):
        #Extrahiere die aktuelle Orientierung aus der Odometrie
        orientation = msg.pose.pose.orientation
        _, _, self.current_angle = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

    def goal_callback(self, goal_request):
        goal_request.velocity = float("%.2f" % goal_request.velocity)
        self.get_logger().info('Recieved goal to start driving with velocity: ' + str(goal_request.velocity))

        if goal_request.velocity > 0.2:
            goal_request.velocity = 0.2
            self.get_logger().info('Velocity too high, defauling to 0.2')
        

        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()
    
    def cancel_callback(self):
        self.get_logger().info('Cancelling goal, stopping robot')

        self.cmd_move.linear.x = 0.0
        self.cmd_move.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_move)

        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting to drive with velocity: ' + str(goal_handle.request.velocity))
        self.mode_selection.set_idling()
        while True:
            try:
                recieved, msg = wait_for_message(Odometry, PipeServer(), 'odom', qos_profile=1)
                if recieved is False:
                    print("hyelp")
                orientation = msg.pose.pose.orientation
                _, _, theta = euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
                ])
                print("msg recieved")
            except:
                time.sleep()
                #print("no single message recieved")
            else:
                self.mode_selection.set_target(theta)
                break

        
        while rclpy.ok():
            try:
                (id, offset, distance) = self.data_tuple
            except:
                time.sleep(0)
                #print("no data from subscriber")       
            else:
                #print(int(id))
                #print(offset)
                #print(distance)

                
                mode = int(id)
                if mode in range(3,999,1):
                    mode = 0
                    print("mode reset")
                print(mode)
                
                try:
                    if mode == 1:
                        self.mode_selection.set_turn()
                    elif mode == 2:
                        self.mode_selection.set_drive()
                    elif mode == 0:
                        self.mode_selection.set_idling()
                    cmd_move, success = self.mode_selection.select_mode(self.data_tuple, goal_handle.request.velocity, self.current_angle)
                except:
                    print("shit's gonked choom")
                    self.cmd_move.linear.x = 0.0
                    self.cmd_move.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_move)
                else:
                    if success:
                        print("success")
                        self.mode_selection.set_idling()
                        cmd_move.linear.x = 0.0
                        cmd_move.angular.z = 0.0
                        self.cmd_pub.publish(cmd_move)
                        break
                    self.cmd_pub.publish(cmd_move)
            
            finally:
                
                time.sleep(0.05)

        return self.determine_action_result(goal_handle)
    
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