import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArucoListener(Node):

    def __init__(self):
        super().__init__('aruco_listener')
        self.subscription_data = self.create_subscription(String, 'data', self.listener_data, 10)
        self.subscription_data      #verhindert "unused variable" error
    
    def listener_data(self, msg):
        data = msg.data
        data_tuple = tuple(map(float, data.split()))        #!!WICHTIG!! konvertiert den input string wieder in ein tuple!!
        print(data_tuple)



def main(args=None):
    rclpy.init()
    aruco_listener = ArucoListener()
    rclpy.spin(aruco_listener)   

    aruco_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


