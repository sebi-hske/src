import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32

class ArucoListener(Node):

    def __init__(self):
        super().__init__('aruco_listener')
        self.subscription_id = self.create_subscription(String, 'id_dst', self.listener_id, 10)
        self.subscription_offset = self.create_subscription(Float32, 'center_offset', self.listener_offset, 10)

        self.subscription_id
        self.subscription_offset   #prevent unused variable warning
    
    def listener_id(self, msg):
        self.get_logger().info('combined id and distance: "%s"' % msg.data)

    def listener_offset(self, msg):
        self.get_logger().info('offset from middle of screen: "%s"' % msg.data)

def main(args=None):
    rclpy.init()
    aruco_listener = ArucoListener()
    rclpy.spin(aruco_listener)   

    aruco_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


