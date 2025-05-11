import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd

class Subscriber(Node): 
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_sub')
        self.subscription = self.create_subscription(
            RobotCmd,
            'cmd_accel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(msg.accel_x)
        self.get_logger().info(msg.accel_y)
        self.get_logger().info(msg.accel_z)
        self.get_logger().info(msg.activate_gripper)

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':      
    main()  