import rclpy
from rclpy.node import Node

from ro45_portalrobot_interfaces.msg import RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos


class DoubleIntegrator(Node):
    def __init__(self):
        super().__init__('double_integrator')
        self.subscription = self.create_subscription(
            RobotCmd,
            'robot_command',
            self.accel_callback,
            10
        )
        self.publisher_ = self.create_publisher(RobotPos, 'robot_position', 10)

        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.accel_scale = 0.2    

    def accel_callback(self, msg:RobotCmd):
       
        self.accel_x = msg.accel_x * self.accel_scale
        self.accel_y = msg.accel_y * self.accel_scale
        self.accel_z = msg.accel_z * self.accel_scale

    def timer_callback(self):
        
        self.vel_x = (self.vel_x + self.accel_x * self.timer_period) 
        self.vel_y = (self.vel_y + self.accel_y * self.timer_period)
        self.vel_z = (self.vel_z + self.accel_z * self.timer_period) 

        
        self.pos_x += self.vel_x * self.timer_period
        self.pos_y += self.vel_y * self.timer_period
        self.pos_z += self.vel_z * self.timer_period

        position_msg = RobotPos()
        position_msg.pos_x = self.pos_x
        position_msg.pos_y = self.pos_y
        position_msg.pos_z = self.pos_z
        self.publisher_.publish(position_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        double_integrator = DoubleIntegrator()

        rclpy.spin(double_integrator)
    except KeyboardInterrupt:
        print("\n Double Integrator Node stopped by user.")
    finally:
        double_integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()