import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotPos

class PositionInput(Node):
    def __init__(self):
        super().__init__('position_input')
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.publisher = self.create_publisher(RobotPos, 'user_input_position', 10)
        self.msg = RobotPos()

    def timer_callback(self):
        self.get_position()
        
    def get_position(self):
        try:
            pos_x_input = input("Enter new value X or press Enter to keep: ")
            pos_y_input = input("Enter new value Y or press Enter to keep: ")
            pos_z_input = input("Enter new value Z or press Enter to keep: ")

           
            if pos_x_input.strip():
                self.msg.pos_x = float(pos_x_input)
            if pos_y_input.strip():
                self.msg.pos_y = float(pos_y_input)
            if pos_z_input.strip():
                self.msg.pos_z = float(pos_z_input)
            
            print("-------------------------------------------")

        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values.")

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