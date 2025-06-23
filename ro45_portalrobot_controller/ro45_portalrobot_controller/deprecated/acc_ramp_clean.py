import rclpy
from rclpy.node import Node
import time

from ro45_portalrobot_interfaces.msg import RobotCmd


class Publisher(Node):
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_pub')
        self.publisher_ = self.create_publisher(RobotCmd, 'robot_command', 10)

        self.min_value = 0.00  # Minimum value for the ramp
        self.max_value = 0.05  # Maximum value for the ramp
        self.ramp_up_time = 1  # Time to ramp up (seconds)
        self.ramp_down_time = 1  # Time to ramp down (seconds)
        self.step_time = 0.1  # Time between steps (seconds)

        self.ramp()

    def ramp_cycle(self, msg, start_val, end_val, ramp_time):
        steps = int(ramp_time / self.step_time)
        step_size = (end_val - start_val) / steps

        for i in range(steps + 1):
            msg.accel_z = start_val + i * step_size
            msg.activate_gripper = True
            self.publisher_.publish(msg)
            time.sleep(self.step_time)

    def ramp(self):
        msg = RobotCmd()
        msg.accel_z = 0.0
        msg.activate_gripper = False

        # Complete ramp cycle
        self.ramp_cycle(msg, self.min_value, self.max_value, self.ramp_up_time)   # Up positive
        self.ramp_cycle(msg, self.max_value, self.min_value, self.ramp_down_time) # Down positive
        self.ramp_cycle(msg, -self.min_value, -self.max_value, self.ramp_up_time) # Up negative
        self.ramp_cycle(msg, -self.max_value, -self.min_value, self.ramp_down_time) # Down negative


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    time.sleep(publisher.ramp_up_time + publisher.ramp_down_time)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()