import rclpy
from rclpy.node import Node
import time

from ro45_portalrobot_interfaces.msg import RobotCmd


class Publisher(Node):
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_pub')
        self.publisher_ = self.create_publisher(RobotCmd, 'cmd_accel', 10)


        self.min_value = 0.0  # Minimum value for the ramp
        self.max_value = 0.1  # Maximum value for the ramp
        self.ramp_up_time = 2  # Time to ramp up (seconds)
        self.ramp_down_time = 2  # Time to ramp down (seconds)
        self.step_time = 0.1  # Time between steps (seconds)

        self.ramp()

    def ramp(self):
        msg = RobotCmd()
        msg.accel_x = 0.0
        msg.accel_y = 0.0
        msg.accel_z = 0.0
        msg.activate_gripper = False

        steps = int(self.ramp_up_time / self.step_time)
        step_size = (self.max_value - self.min_value) / steps
        for i in range(steps + 1):
            msg.accel_x = self.min_value + i * step_size
            self.publisher_.publish(msg)
            time.sleep(self.step_time)

        steps = int(self.ramp_down_time / self.step_time)
        step_size = (self.max_value - self.min_value) / steps
        for i in range(steps + 1):
            msg.accel_x = self.max_value - i * step_size
            self.publisher_.publish(msg)
            time.sleep(self.step_time)

        steps = int(self.ramp_up_time / self.step_time)
        step_size = (self.max_value - self.min_value) / steps
        for i in range(steps + 1):
            msg.accel_x = -self.min_value - i * step_size
            self.publisher_.publish(msg)
            time.sleep(self.step_time)

        steps = int(self.ramp_down_time / self.step_time)
        step_size = (self.max_value - self.min_value) / steps
        for i in range(steps + 1):
            msg.accel_x = -self.max_value + i * step_size
            self.publisher_.publish(msg)
            time.sleep(self.step_time)


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    time.sleep(publisher.ramp_up_time + publisher.ramp_down_time)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()