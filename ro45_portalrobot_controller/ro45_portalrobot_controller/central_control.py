import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos
from std_msgs.msg import Float32
from pd_regler import PDRegler
from ro45_action_interfaces.action import MovetoPos
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

P_VALUE_X = 0.15
D_VALUE_X = 1.5 # up from 0.7 default, possible delay in execute callback

P_VALUE_Y = 0.08
D_VALUE_Y = 1.1

P_VALUE_Z = 0.4 #0.4
D_VALUE_Z = 2.8 #1.9

class CentralControl(Node):
    def __init__(self):
        super().__init__('central_control')
        self.subscription_pos = self.create_subscription(
            RobotPos,
            'robot_position',
            self.pos_callback,
            10
        )
        
        #only for testing
        """
        self.subscription_poscmd = self.create_subscription(
            RobotPos,
            'user_input_position',
            self.cmd_callback,
            10
            )
        """
        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            MovetoPos,
            'move_to_position',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.publisher_cmd = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.msg = RobotCmd()
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0      
        self.msg.accel_z = 0.0
        self.msg.activate_gripper = False

        self.goal_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        #initialize PD controller
        self.pd_control_x = PDRegler(P_VALUE_X, D_VALUE_X)   
        self.pd_control_y = PDRegler(P_VALUE_Y, D_VALUE_Y)
        self.pd_control_z = PDRegler(P_VALUE_Z, D_VALUE_Z)
        
        self.calibration()

    def execute_callback(self, goal_handle):
        try:
            self.get_logger().info("Executing goal...")
            self.timer_period = 0.01
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            goal_handle.succeed()    
            result = MovetoPos.Result()    
            return result
        except Exception as e:
            self.get_logger().error(f"Error during goal execution: {e}")
            goal_handle.abort()
            self.failsafe_hold_pos()
            return None
    
    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request to move to position: " + str(goal_request))
        self.desired_pos_x = (goal_request.position_x - self.corr_val_x) * -1.0
        self.desired_pos_y = (goal_request.position_y - self.corr_val_y) * -1.0
        self.desired_pos_z = (goal_request.position_z - self.corr_val_z) # * -1.0
        self.get_logger().info("corrected positions for robot "+  str(self.desired_pos_x)+str(self.desired_pos_y)+str(self.desired_pos_z))
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
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
        self.timer.cancel()
        self.get_logger().warn("Failsafe activated. Holding position.")
        self.desired_pos_x, self.desired_pos_y, self.desired_pos_z = self.pos_x, self.pos_y, self.pos_z
        if (self.pos_z + self.corr_val_z) > 0.05:
            self.desired_pos_z = 0.05 - self.corr_val_z
        self.failsafe_timer = self.create_timer(0.1, self.timer_callback)
        

        

    def calibration(self):
        #implement calibration for all 3 axis (x,y,z)
        #step all axis to zero position
        #set all position values to zero
        self.msg.accel_x = 0.01     #positive values
        self.msg.accel_y = 0.002    #positive values
        self.msg.accel_z = -0.002      #negative values
        self.publish_command() 

        self.calibration_wait_time = 20.0    #set time to wait for calibration
        self.calibration_elapsed_time = 0.0
        self.calib_timer = 0.1
        self.calibration_timer = self.create_timer(self.calib_timer, self.calibration_callback)

    def calibration_callback(self):
        self.calibration_elapsed_time += self.calib_timer
        self.get_logger().info("Waiting for calibration to finish...")
        if self.calibration_elapsed_time >= self.calibration_wait_time:
            self.calibration_timer.cancel()

            #set values to zero for non simulation runs
            self.set_zero()
            
            self.set_correction_values()
            print(self.corr_val_x, self.corr_val_y, self.corr_val_z)
            self.get_logger().info("Calibration finished. Robot is in zero position.")
            #self.timer_period = 0.1
            #self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def set_correction_values(self):
        self.corr_val_x = self.pos_x
        self.corr_val_y = self.pos_y
        self.corr_val_z = self.pos_z

    def timer_callback(self):
        u_x, u_y, u_z = self.call_pd_controller()
        self.msg.accel_x = u_x
        self.msg.accel_y = u_y
        self.msg.accel_z = u_z
        self.publish_command()
           
    def pos_callback(self, msg):
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.pos_z = msg.pos_z

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
        self.get_logger().info("Publishing -> accel_x: {:.6f}".format(self.msg.accel_x) +
                               ", accel_y: {:.6f}".format(self.msg.accel_y) +
                               ", accel_z: {:.6f}".format(self.msg.accel_z))

    


def main(args=None):
    print("Starting Central Control Node")
    rclpy.init(args=args)

    try:
        central_control = CentralControl()
        mt_executor = MultiThreadedExecutor()
        rclpy.spin(central_control, executor=mt_executor)
    except KeyboardInterrupt:
        pass

    central_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()
        