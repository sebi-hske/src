import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class PositionInput(Node):
    def __init__(self):
        super().__init__('position_input')
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.publisher = self.create_publisher(Float32, 'position_command', 10)
        self.msg = Float32()

    def timer_callback(self):
        self.get_position()
        
    def get_position(self):
        try:
            pos_cmd = input(f"Enter desired position: ")
           
            if pos_cmd.strip():
                self.msg.data = float(pos_cmd)
            
        except ValueError:
            self.get_logger().warn("Invalid input. Please enter numeric values.")

        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    publisher = PositionInput()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()