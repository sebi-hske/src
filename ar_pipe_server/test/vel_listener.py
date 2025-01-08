import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelListener(Node):

    def __init__(self):
        super().__init__('aruco_listener')
        self.subscription_data = self.create_subscription(Twist, 'cmd_vel', self.listener_data, 10)
        self.subscription_data      #verhindert "unused variable" error
    
    def listener_data(self, msg):
       x = msg.linear.x 
       z = msg.angular.z
       print("linear " + str(x))
       print("angular " + str(z))




def main(args=None):
    rclpy.init()
    vel_listener = VelListener()
    rclpy.spin(vel_listener)   

    vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


