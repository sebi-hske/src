import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        self.subscription_data = self.create_subscription(Odometry, 'odom', self.listener_data, 10)
        self.subscription_data      #verhindert "unused variable" error
    
    def listener_data(self, msg):        
        #Extrahiere die aktuelle Orientierung aus der Odometrie
        orientation = msg.pose.pose.orientation
        _, _, self.current_angle = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        print(self.current_angle)


def main(args=None):
    rclpy.init()
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)   

    odom_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

