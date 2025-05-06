import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotPos

class Subscriber(Node): 
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_sub')
        self.subscription = self.create_subscription(
            RobotPos,
            'position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Robot Position -> x: {msg.pos_x:.2f}, y: {msg.pos_y:.2f}, z: {msg.pos_z:.2f}")

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':      
    main()