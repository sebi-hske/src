import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotPos

class Subscriber(Node): 
    def __init__(self):
        super().__init__('subscriber_position_input')
        self.subscription = self.create_subscription(
            RobotPos,
            'user_position_input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Input Position -> x: {msg.pos_x:.4f}, y: {msg.pos_y:.4f}, z: {msg.pos_z:.4f}")



def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':      
    main()  