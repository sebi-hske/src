from ro45_portalrobot_interfaces.msg import RobotCmd, RobotPos
from std_msgs.msg import String
from rclpy.node import Node
import math
import time


class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')

        
        self.publisher_cmd = self.create_publisher(RobotCmd, 'robot_command', 10)
        
        self.subscription = self.create_subscription(RobotPos, 'robot_position', self.position_callback, 10)
        self.subscription_d = self.create_subscription(String, 'desired_position', self.desire_callback, 10)
        
        self.current_position = RobotPos()
        
        self.pos_msg = String()


        
        self.acceleration = 0.2  # Constant acceleration value
        self.timer_period = 0.1  # Timer period in seconds
        self.timer = self.create_timer(self.timer_period, self.update_acceleration)

        # State variables
        self.time_remaining_x = 0.0
        

    def position_callback(self, msg):
        self.current_position = msg



    def desire_callback(self, msg):
        self.desired_position = float(msg.data)

        
        self.time_remaining_x = self.calculate_time(self.desired_position.pos_x, self.current_position.pos_x)
        

    def calculate_time(self, desired, current):
        
        distance = desired - current

        
        if distance == 0:
            return 0.0

        
        try:
            time_required = math.sqrt(abs(2 * distance / self.acceleration))
            return time_required
        except ValueError:
            self.get_logger().warn("Invalid calculation for time. Check inputs.")
            return 0.0

    def update_acceleration(self):
        # Initialize acceleration commands
        cmd_msg = RobotCmd()
        cmd_msg.accel_x = 0.0
        cmd_msg.accel_y = 0.0
        cmd_msg.accel_z = 0.0

        
        while self.time_remaining_x > 0:
            cmd_msg.accel_x = self.acceleration if self.desired_position.pos_x > self.current_position.pos_x else -self.acceleration

            self.publisher_cmd.publish(cmd_msg)
            time.sleep(self.time_remaining_x/4)

            cmd_msg.accel_x = -cmd_msg.accel_x
            self.publisher_cmd.publish(cmd_msg)
            time.sleep(self.time_remaining_x/2)

            cmd_msg.accel_x = -cmd_msg.accel_x
            self.publisher_cmd.publish(cmd_msg)
            time.sleep(self.time_remaining_x/4)
            self.publisher_cmd.publish(cmd_msg)



            self.time_remaining_x -= self.timer_period

        
        cmd_msg.accel_x = 0.0
        cmd_msg.accel_y = 0.0
        cmd_msg.accel_z = 0.0
        self.publisher_cmd.publish(cmd_msg)


        
        

        # Log the output
        self.get_logger().info(
            f"Accel -> pos_x: {cmd_msg.accel_x:.2f}, pos_y: {cmd_msg.accel_y:.2f}, pos_z: {cmd_msg.accel_z:.2f}"
        )


def main(args=None):
    import rclpy
    rclpy.init(args=args)

    position_controller = PositionController()

    try:
        rclpy.spin(position_controller)
    except KeyboardInterrupt:
        pass

    position_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()