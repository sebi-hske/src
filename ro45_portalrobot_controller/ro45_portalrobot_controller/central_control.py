import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos
from std_msgs.msg import Float32
from pd_regler import PDRegler
from ro45_action_interfaces.action import MovetoPos

P_VALUE = 2.0
D_VALUE = 1.0

class CentralControl(Node):
    def __init__(self):
        super().__init__('central_control')
        self.subscription_pos = self.create_subscription(
            RobotPos,
            'robot_position',
            self.pos_callback,
            10
        )

        self.subscription_poscmd = self.create_subscription(   #only for testing
            Float32,
            'position_command',
            self.cmd_callback,
            10
            )

        self.publisher_pos = self.create_publisher(RobotPos, 'robot_position', 10)
        self.publisher_cmd = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.msg = RobotCmd()

        self.pd_control = PDRegler(P_VALUE, D_VALUE)        #initialize PD controller
        
        self.calibration()
           
    def calibration(self):
        #implement calibration for all 3 axis (x,y,z)
        #step all axis to zero position
        #set all position values to zero
        self.calib_timer = 0.1
        self.calibration_timer = self.create_timer(self.calib_timer, self.calibration_callback)

        self.msg.accel_x = 0.2
        self.msg.accel_y = 0.0      #!! set direction !!
        self.msg.accel_z = 0.0      #!! set direction !!
        self.publish_command()

        self.calibration_wait_time = 1.0    #set time to wait for calibration
        self.calibration_elapsed_time = 0.0

    def calibration_callback(self):
        self.calibration_elapsed_time += self.calib_timer
        self.get_logger().info("Waiting for calibration to finish...")
        if self.calibration_elapsed_time >= self.calibration_wait_time:
            self.calibration_timer.cancel()
            self.msg.accel_x = 0.0
            self.msg.accel_y = 0.0
            self.msg.accel_z = 0.0
            self.publish_command()
            pos_msg = RobotPos()
            pos_msg.pos_x = 0.0
            pos_msg.pos_y = 0.0
            pos_msg.pos_z = 0.0
            self.publisher_pos.publish(pos_msg)
            self.get_logger().info("Calibration finished. Robot is in zero position.")
            self.timer_period = 0.1
            self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        fault = self.call_pd_controller()
        self.msg.accel_x = fault
        self.publish_command()
           
    def pos_callback(self, msg):
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.pos_z = msg.pos_z

    def cmd_callback(self, msg):        #only for testing
        self.desired_pos_x = msg.data

    def call_pd_controller(self):
        try:  
            fault = self.pd_control.berechne(self.desired_pos_x, self.pos_x, self.timer_period)
        except AttributeError:
            self.get_logger().warn("Desired position not set. Please set the desired position first.")
            fault = 0.0
        return fault
    
    def publish_command(self):
        self.get_logger().info("Publishing -> accel_x: {:.2f}".format(self.msg.accel_x) +
                               ", accel_y: {:.2f}".format(self.msg.accel_y) +
                               ", accel_z: {:.2f}".format(self.msg.accel_z))
        self.publisher_cmd.publish(self.msg)

    


def main(args=None):
    rclpy.init(args=args)

    central_control = CentralControl()

    try:
        rclpy.spin(central_control)
    except KeyboardInterrupt:
        pass

    central_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()
        