import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos
from std_msgs.msg import Float32
from pd_regler import PDRegler

P_VALUE = 2.0
D_VALUE = 1.0

class CentralControl(Node):
    def __init__(self):
        super().__init__('central_control')
        self.subscription = self.create_subscription(
            RobotPos,
            'robot_position',
            self.pos_callback,
            10
        )

        self.subscription = self.create_subscription(
            Float32,
            'position_command',
            self.cmd_callback,
            10
            )

        self.publisher = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.msg = RobotCmd()
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0
        self.msg.accel_z = 0.0
        self.msg.activate_gripper = False

        self.pd_control = PDRegler(P_VALUE, D_VALUE)        #initialize PD controller

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.calibration()  

    def calibration(self):
        #implement calibration for all 3 axis (x,y,z)
        #step all axis to zero position
        #set all position values to zero
        pass

    def pos_callback(self, msg):
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.pos_z = msg.pos_z

    def cmd_callback(self, msg):
        self.desired_pos_x = msg.data

    def call_pd_controller(self):
        try:  
            fault = self.pd_control.berechne(self.desired_pos_x, self.pos_x, self.timer_period)    #set desired and actual value
        except AttributeError:
            self.get_logger().warn("Desired position not set. Please set the desired position first.")
            fault = 0.0
        return fault

    def timer_callback(self):
        fault = self.call_pd_controller()
        self.msg.accel_x = fault
        self.get_logger().info("Publishing -> accel_x: {:.2f}".format(self.msg.accel_x))
        self.publisher.publish(self.msg)


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
        