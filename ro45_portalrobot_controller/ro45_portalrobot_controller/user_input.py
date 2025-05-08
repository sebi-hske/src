import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotCmd


class Publisher(Node):
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_pub')
        self.publisher_ = self.create_publisher(RobotCmd, 'RobotCmd', 10)
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg = RobotCmd()
        self.msg.accel_x = 0.0
        self.msg.accel_y = 0.0
        self.msg.accel_z = 0.0
        self.msg.activate_gripper = False

    def timer_callback(self):
        try:
            accel_x_input = input(f"Current accel_x: {self.msg.accel_x:.2f}. Enter new value (or press Enter to keep): ")
            accel_y_input = input(f"Current accel_y: {self.msg.accel_y:.2f}. Enter new value (or press Enter to keep): ")
            accel_z_input = input(f"Current accel_z: {self.msg.accel_z:.2f}. Enter new value (or press Enter to keep): ")

           
            if accel_x_input.strip():
                self.msg.accel_x = float(accel_x_input)
            if accel_y_input.strip():
                self.msg.accel_y = float(accel_y_input)
            if accel_z_input.strip():
                self.msg.accel_z = float(accel_z_input)

        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values.")

       
        self.get_logger().info(
            f"Publishing -> accel_x: {self.msg.accel_x:.2f}, accel_y: {self.msg.accel_y:.2f}, accel_z: {self.msg.accel_z:.2f}"
        )
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()