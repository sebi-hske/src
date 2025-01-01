import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from ro36_interfaces.action import GoTo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from ro36_simple_mover_server.simple_robot_mover import SimpleRobotMover


class Ro36SimpleMoverServer(Node):
   def __init__(self):
        super().__init__('ro36_simple_mover_server')
        self._last_pose_x = None
        self._last_pose_y = None
        self._last_pose_theta = None
        self.cmd_move = Twist()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            GoTo,
            'go_to',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback)
        
        self.callback_group=ReentrantCallbackGroup()
        self._goal_lock = threading.Lock()
        self.simple_robot_mover = SimpleRobotMover()


   def _odom_callback(self, msg):
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y
        _, _, self._last_pose_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        )

   def _goal_callback(self, goal_request): # = default behavior + message (could be omitted)
        self.get_logger().info('Received goal request with target pose ' +
                               _pose_as_string(goal_request.pose))
        
        if self._last_pose_x is None:
            self.get_logger().info('No initial pose, Goal is invalid')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

   def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

   def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling move to pose ' +
                               _pose_as_string(goal_handle.request.pose))
        self.cmd_move.linear.x = 0.0
        self.cmd_move.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_move)

        return CancelResponse.ACCEPT

   def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing move to pose ' +
                               _pose_as_string(goal_handle.request.pose))
        
        if self._last_pose_x is not None:
            
            self.simple_robot_mover.set_target_pose(goal_handle.request.pose.x, goal_handle.request.pose.y, goal_handle.request.pose.theta)
           
            
            while rclpy.ok():
                velocity_list = self.simple_robot_mover.get_velocity_command(self._last_pose_x, self._last_pose_y, self._last_pose_theta)
                print(velocity_list)

                if velocity_list == None:
                    self.cmd_move.linear.x = 0.0
                    self.cmd_move.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_move)
                    print('we did it')
                    break
                
                self.cmd_move.linear.x = velocity_list[0]
                self.cmd_move.angular.z = velocity_list[1]
                self.cmd_pub.publish(self.cmd_move)

                time.sleep(0.1)       #möglichst niedrig

            feedback_msg = GoTo.Feedback()
            #feedback_msg.dist_to_goal = self.simple_robot_mover._calculate_distance_to_goal(goal_handle.request.pose)
            goal_handle.publish_feedback(feedback_msg)
            return self._determine_action_result(goal_handle)   #self raus


   def _determine_action_result(self, goal_handle):     #beim einbinden in class definition warning von _execute_callback
        result = GoTo.Result()
        if goal_handle.is_active:
            self.get_logger().info('Move to pose ' +
                                   _pose_as_string(goal_handle.request.pose) +
                                   ' succeeded.')
            goal_handle.succeed()
            result.reached = True
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Move to pose ' +
                                   _pose_as_string(goal_handle.request.pose) +
                                   ' was cancelled.')
        else:
            if goal_handle.is_active():
                goal_handle.abort()
            self.get_logger().info('Move to pose ' +
                                   _pose_as_string(goal_handle.request.pose) +
                                   ' was aborted.')
        return result

def _pose_as_string(pose):
    return 'x={}, y={}, theta={}'.format(pose.x, pose.y, pose.theta)


def main():
    print('Action Server active')
    rclpy.init()
    try:
        simple_mover_server = Ro36SimpleMoverServer()

        mt_executer = MultiThreadedExecutor()
        rclpy.spin(simple_mover_server, executor=mt_executer)
        

        simple_mover_server.destroy()
    finally:
        rclpy.shutdown()