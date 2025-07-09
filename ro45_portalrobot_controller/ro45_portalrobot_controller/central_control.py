import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos
from std_msgs.msg import Float32
from pd_regler import PDRegler
from ro45_action_interfaces.action import MovetoPos
from ro45_action_interfaces.action import Intercept
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
import math
import subprocess
from pathlib import Path
import time

P_VALUE_X = 0.7
D_VALUE_X = 3.0 # up from 0.7 default, possible delay in execute callback

P_VALUE_Y = 0.9
D_VALUE_Y = 7.0

P_VALUE_Z = 0.8 #0.4
D_VALUE_Z = 3.0 #1.9

STARTING_Z = -0.05
INTERCEPT_Z = -0.06 
PICKUP_Z = -0.08
AFTER_PICKUP = -0.06  
DROPOFF_Z = -0.05

BIN_1_X = 0.145
BIN_Y = 0.12

BIN_2_X = 0.215

DROP_TOLERANCE = 0.01

class CentralControl(Node):
    def __init__(self):
        super().__init__('central_control')

        self.subscription_pos = self.create_subscription(
            RobotPos,
            'robot_position',
            self.pos_callback,
            10
        )
        
        self.goal_handle = None
        self.position_action_server = ActionServer(
            self,
            MovetoPos,
            'move_to_position',
            execute_callback=self.position_execute_callback,
            goal_callback=self.position_goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        self.intercept_action_server = ActionServer(
            self,
            Intercept,
            'intercept_object',
            execute_callback=self.intercept_execute_callback,
            goal_callback=self.intercept_goal_callback,
            handle_accepted_callback=self.handle_accepted_callback_intercept,
            cancel_callback=self.cancel_callback
        )

        self.action_client = ActionClient(self, MovetoPos, 'move_to_position')
        
        self.publisher_cmd = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.msg = RobotCmd()
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0      
        self.msg.accel_z = 0.0
        self.msg.activate_gripper = False
        self.corr_val_x = 0.0
        self.corr_val_y = 0.0
        self.corr_val_z = 0.0

        self.goal_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        self.pd_control_x = PDRegler(P_VALUE_X, D_VALUE_X)   
        self.pd_control_y = PDRegler(P_VALUE_Y, D_VALUE_Y)
        self.pd_control_z = PDRegler(P_VALUE_Z, D_VALUE_Z)
        self.no_of_timers = 0
        self.timer_period = 0.1
        time.sleep(2)  
        self.calibration()
    

    def position_execute_callback(self, goal_handle):
        if hasattr(self, 'failsafe_timer'):
            self.failsafe_timer.destroy() #TODO: documentation for rclpy.Time
            self.no_of_timers -= 1
            self.get_logger().info("Failsafe timer cancelled.")
        try:
            self.get_logger().info("Executing moving goal...")
            if hasattr(self, 'timer'):
                self.get_logger().info("had to destroy movement timer")
                self.timer.destroy()
                self.no_of_timers -= 1
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.no_of_timers += 1
            goal_handle.succeed()    
            result = MovetoPos.Result()    
            return result
        except Exception as e:
            self.get_logger().error(f"Error during moving execution: {e}")
            goal_handle.abort()
            self.failsafe_hold_pos()
            return None
        
    def intercept_execute_callback(self, goal_handle):
        if hasattr(self, 'failsafe_timer'):
            self.failsafe_timer.destroy()
            self.no_of_timers -= 1
            self.get_logger().info("Failsafe timer cancelled.")
        if hasattr(self, 'timer'):
            self.get_logger().info("had to destroy movement timer")
            self.timer.destroy()
            self.no_of_timers -= 1
        try:
            self.get_logger().info("Executing intercept goal...")
            self.goal_handle = goal_handle
            self.send_moving_goal(self.pickup_x, self.pickup_y, INTERCEPT_Z)            
            self.initialize_countdown()
            self.not_picked = True
            self.pickup_timer = self.create_timer(self.timer_period, self.pickup_sequence)
            self.no_of_timers += 1
            goal_handle.succeed()
            result = Intercept.Result()
            return result
        except Exception as e:
            self.get_logger().error(f"Error during intercept execution: {e}")
            goal_handle.abort()
            self.failsafe_hold_pos()
            return None        
    
    def initialize_countdown(self):
        self.remaining_time = float(self.time_to_intercept)
        self.get_logger().info(f"Starting countdown: {self.remaining_time} seconds")
    
    def pickup_sequence(self):
        movement_start_time = 2.0
        self.remaining_time -= self.timer_period
        if (self.remaining_time <= movement_start_time) and (self.remaining_time >= movement_start_time - self.timer_period):
            self.msg.activate_gripper = True
            self.send_moving_goal(self.pickup_x, self.pickup_y , PICKUP_Z)
            self.get_logger().info("Waiting for timer to run out")            
        if (self.remaining_time <= movement_start_time) and (self.corrected_output_z <= (PICKUP_Z + 0.002)) and self.not_picked:
            self.get_logger().info("Pickup sequence complete!")
            self.not_picked = False
            self.send_moving_goal(self.pickup_x, self.pickup_y ,DROPOFF_Z)
        if (self.remaining_time <= 0.0) and (self.corrected_output_z >= AFTER_PICKUP):
            self.get_logger().info("Moving to drop-off position after pickup")
            self.pickup_timer.destroy()
            self.no_of_timers -= 1
            self.drop_in_bin()

    def drop_in_bin(self):
        self.get_logger().info("Dropping object in bin...")
        
        if int(self.object_class) == 1:  
            self.get_logger().info("Moving to bin position for object type 1")
            self.send_moving_goal(BIN_1_X, BIN_Y, DROPOFF_Z)
        
        elif int(self.object_class) == 2:
            self.get_logger().info("Moving to bin position for object type 2")
            self.send_moving_goal(BIN_2_X, BIN_Y, DROPOFF_Z)
            
        elif int(self.object_class) == 0:
            self.get_logger().warn(f"Unknown object class: {self.object_class}")
            self.failsafe_hold_pos()

    def goto_start_position(self):
        self.get_logger().info("Moving to starting position")
        self.send_moving_goal(0.19, 0.055, STARTING_Z)

    def send_moving_goal(self, x, y, z):
        self.get_logger().info(f"Sending moving goal to position: x={x}, y={y}, z={z}")
        goal_msg = MovetoPos.Goal()
        goal_msg.position_x = x
        goal_msg.position_y = y
        goal_msg.position_z = z
    
        if not self.action_client.wait_for_server(5.0):
            self.get_logger().error("Action server not available. Cannot send goal.")
            self.failsafe_hold_pos()
            return False
        try:       
            goal_future = self.action_client.send_goal_async(goal_msg)
        
            """
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                self.failsafe_hold_pos()
                return False
            """
            return True
            
        except Exception as e:
            self.get_logger().error(f'Move failed with error: {str(e)}')
            self.failsafe_hold_pos()
            return False
    
    
    def position_goal_callback(self, goal_request):
        self.get_logger().info("Received goal request to move to position: " + str(goal_request))
        self.desired_pos_x = (goal_request.position_x - self.corr_val_x) * -1.0
        self.desired_pos_y = (goal_request.position_y - self.corr_val_y) * -1.0
        self.desired_pos_z = (goal_request.position_z - self.corr_val_z)  * -1.0
        self.get_logger().info("corrected positions for robot "+  str(self.desired_pos_x)+str(self.desired_pos_y)+str(self.desired_pos_z))
        
        return GoalResponse.ACCEPT
    
    def intercept_goal_callback(self, goal_request):
        self.get_logger().info("Received goal request to intercept object at position: " + str(goal_request))
        self.desired_pos_x = (goal_request.position_x - self.corr_val_x) * -1.0
        self.desired_pos_y = (goal_request.position_y - self.corr_val_y) * -1.0
        self.desired_pos_z = (INTERCEPT_Z - self.corr_val_z) * -1.0
        self.time_to_intercept = goal_request.time
        self.object_class = goal_request.object_class
        self.pickup_x, self.pickup_y = goal_request.position_x, goal_request.position_y
        self.get_logger().info("intercepting object at position: " + str(self.desired_pos_x) + ", " + str(self.desired_pos_y) + " in " + str(self.time_to_intercept) + " seconds")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
                #self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def handle_accepted_callback_intercept(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Adding Object to target list.')
                self.goal_handle.abort()            
            self.goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancelled. Stopping execution")
        self.failsafe_hold_pos()
        goal_handle.canceled()
        return CancelResponse.ACCEPT
           
    def set_zero(self):
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0
        self.msg.accel_z = 0.0
        self.publish_command()

    def revert_last_cmd(self):
        self.msg.accel_x = self.msg.accel_x * -1.0
        self.msg.accel_y = self.msg.accel_y * -1.0
        self.msg.accel_z = self.msg.accel_z * -1.0
        self.publish_command()

    def failsafe_hold_pos(self):
        #hold position in case of emergency
        try:
            self.timer.destroy()
            self.no_of_timers -= 1
        except:
            self.get_logger().warn("No timer active")
        self.get_logger().warn("Failsafe activated. Holding position.")
        self.desired_pos_x, self.desired_pos_y, self.desired_pos_z = self.pos_x, self.pos_y, self.pos_z
        
        self.desired_pos_z = (STARTING_Z - self.corr_val_z) * -1.0
        self.failsafe_timer = self.create_timer(self.timer_period, self.timer_callback)
        self.no_of_timers += 1        

    def calibration(self):
        #implement calibration for all 3 axis (x,y,z)
        #step all axis to zero position
        #set all position values to zero
        self.msg.accel_x = 0.3     #positive values
        self.msg.accel_y = 0.2    #positive values
        self.msg.accel_z = -0.16      #negative values
        self.publish_command()
        time.sleep(0.1) 
        self.set_zero()

        self.calibration_wait_time = 20.0    #set time to wait for calibration
        self.calibration_elapsed_time = 0.0
        self.calib_timer = 0.1
        self.calibration_timer = self.create_timer(self.calib_timer, self.calibration_callback)
        self.no_of_timers += 1

    def calibration_callback(self):
        self.calibration_elapsed_time += self.calib_timer
        self.get_logger().info("Waiting for calibration to finish...")
        if self.calibration_elapsed_time >= self.calibration_wait_time:
            self.calibration_timer.destroy()
            self.no_of_timers -= 1
            #set values to zero for non simulation runs
            #self.revert_last_cmd()
            
            
            self.set_correction_values()
            print(self.corr_val_x, self.corr_val_y, self.corr_val_z)
            self.get_logger().info("Calibration finished. Robot is in zero position.")
            self.goto_start_position()
            #self.timer_period = 0.1
            #self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def set_correction_values(self):
        self.corr_val_x = self.pos_x
        self.corr_val_y = self.pos_y
        self.corr_val_z = self.pos_z

    def timer_callback(self):
        print(self.no_of_timers)
        u_x, u_y, u_z = self.call_pd_controller()
        self.msg.accel_x = u_x
        self.msg.accel_y = u_y
        self.msg.accel_z = u_z
        if (math.isclose(self.corrected_output_x, BIN_1_X, abs_tol=DROP_TOLERANCE) and math.isclose(self.corrected_output_y, BIN_Y, abs_tol=DROP_TOLERANCE)
                or math.isclose(self.corrected_output_x, BIN_2_X, abs_tol=DROP_TOLERANCE) and math.isclose(self.corrected_output_y, BIN_Y, abs_tol=DROP_TOLERANCE)):
            self.msg.activate_gripper = False
            self.publish_command()            
        else:
            self.msg.activate_gripper = True
            self.publish_command()    
           
    def pos_callback(self, msg):
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.pos_z = msg.pos_z
        
        self.corrected_output_x = (msg.pos_x - self.corr_val_x) * -1.0
        self.corrected_output_y = (msg.pos_y - self.corr_val_y) * -1.0
        self.corrected_output_z = (msg.pos_z - self.corr_val_z) * -1.0

    #only for testing
    """
    def cmd_callback(self, msg):        
        self.desired_pos_x = msg.pos_x - self.corr_val_x
        self.desired_pos_y = msg.pos_y - self.corr_val_y
        self.desired_pos_z = msg.pos_z - self.corr_val_z
    """
    def call_pd_controller(self):
        try:  
            u_x = self.pd_control_x.berechne(self.desired_pos_x, self.pos_x, self.timer_period)
            u_y = self.pd_control_y.berechne(self.desired_pos_y, self.pos_y, self.timer_period)
            u_z = self.pd_control_z.berechne(self.desired_pos_z, self.pos_z, self.timer_period)
        except AttributeError:
            self.get_logger().warn("Desired position not set. Please set the desired position first.")
            u_x = 0.0
            u_y = 0.0
            u_z = 0.0
        return u_x, u_y, u_z
    
    def publish_command(self):
        self.publisher_cmd.publish(self.msg)
        #self.get_logger().info("Publishing -> accel_x: {:.6f}".format(self.msg.accel_x) +
         #                      ", accel_y: {:.6f}".format(self.msg.accel_y) +
          #                     ", accel_z: {:.6f}".format(self.msg.accel_z))

    


def main(args=None):
    print("Starting Central Control Node")
    rclpy.init(args=args)

    try:
        central_control = CentralControl()
        mt_executor = MultiThreadedExecutor()
        rclpy.spin(central_control, executor=mt_executor)
    except KeyboardInterrupt:
        print("Central Control Node interrupted by user.")
    finally:
        central_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  
    main()
        