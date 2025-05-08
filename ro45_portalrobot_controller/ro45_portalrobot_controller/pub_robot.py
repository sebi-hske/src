import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd

class Publisher(Node):
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_pub')
        self.publisher_ = self.create_publisher(RobotCmd, 'RobotCmd', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.1
        self.msg = RobotCmd()
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0
        self.msg.accel_z = 0.0
        self.msg.activate_gripper=False

    def timer_callback(self):
        
        self.msg.accel_x += self.i
        self.msg.accel_y += self.i
        self.msg.accel_z += self.i
        self.msg.activate_gripper=False
        self.publisher_.publish(self.msg)
        

def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()