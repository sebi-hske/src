import rclpy
from rclpy.node import Node
import time

from ro45_portalrobot_interfaces.msg import RobotCmd

class Publisher(Node):
    def __init__(self):
        super().__init__('ro45_portalrobot_controller_pub')
        publisher = self.create_publisher(RobotCmd, 'cmd_accel', 10)

        pos_acc_time = 1
        neg_acc_time = 1
        zero_time = 1
        
        msg = RobotCmd()
        msg.accel_x = 0.0
        msg.accel_y = 0.0
        msg.accel_z = 0.0
        msg.activate_gripper=False

        msg.accel_x = 0.01
        publisher.publish(msg)
        time.sleep(pos_acc_time)

        msg.accel_x = 0.02
        publisher.publish(msg)
        time.sleep(pos_acc_time)
        
        msg.accel_x = 0.05
        publisher.publish(msg)
        time.sleep(pos_acc_time)

        msg.accel_x = 0.02
        publisher.publish(msg)
        time.sleep(pos_acc_time)

        msg.accel_x = 0.01
        publisher.publish(msg)
        time.sleep(pos_acc_time)

        msg.accel_x = 0.0
        publisher.publish(msg)
        time.sleep(zero_time)

        msg.accel_x = -0.01
        publisher.publish(msg)
        time.sleep(neg_acc_time)

        msg.accel_x = -0.02
        publisher.publish(msg)
        time.sleep(neg_acc_time)
        
        msg.accel_x = -0.05
        publisher.publish(msg)
        time.sleep(neg_acc_time)

        msg.accel_x = -0.02
        publisher.publish(msg)
        time.sleep(neg_acc_time)

        msg.accel_x = -0.01
        publisher.publish(msg)
        time.sleep(neg_acc_time)

        msg.accel_x = 0.0
        publisher.publish(msg)
        time.sleep(zero_time)
    
    

def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()