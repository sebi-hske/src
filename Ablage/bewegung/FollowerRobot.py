import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

TURNING_RATE = 3.0      #Regelungsfaktor
MID_GOAL = 0.35  

class FollowerRobot(Node):
    def __init__(self):
        super().__init__('follower_robot')

        #Abonniere die Aruco-Marker-Daten (Tuple: Marker-ID, Offset, Distanz)
        self.subscription_offset = self.create_subscription(
            String,
            'data',
            self.offset_callback,
            10
        )

        #Steuerbefehl publizieren
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #Initialisiere Variablen
        self.offset_tuple = None #Tuple (Marker-ID, Offset, Distanz)
        self.linear_velocity = 0.5 #Maximale Vorwärtsgeschwindigkeit 
        self.kp_distance = 0.1 #Proportionalitätskonstante für Distanz
        self.kp_offset = 0.5 #Proportionalitätskonstante für Offset
        self.desired_distance = 2.0 #Soll-Abstand
        self.distance_threshold = 0.2 #Tolleranz für den Sollabstand
        #timer_period = 0.0001
        #self.timer = self.create_timer(timer_period, self.timer_callback) 

    def offset_callback(self, msg):
        try:
            #Konvertiere Offset-Daten aus String in ein Tupel
            data_tuple = tuple(map(float, msg.data.split()))
            self.offset_tuple = data_tuple
            self.get_logger().info(f"Empfangenes Tuple: {self.offset_tuple}")
            self.follow_target()
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Verarbeitung der Daten: {e}")

    #def timer_callback(self):
        

    def follow_target(self):
        if self.offset_tuple is not None:
            #Tupel extrahieren: (Marker-ID, Offset, Distanz)
            marker_id, offset, distance = self.offset_tuple
           


            #Abstandskontrolle (linear.x)
            distance_error = self.desired_distance - distance
            if abs(distance_error) < self.distance_threshold:
                linear_velocity = 0.0 #Halteposition
            else:
                linear_velocity = -self.kp_distance * distance_error #Anpassung der Geschwindigkeit

            #Offsetkontrolle (angular.z)
            offset = offset * -1.0
            rel_offset = offset / distance
            angular_velocity = MID_GOAL - rel_offset
            angular_velocity = angular_velocity * -TURNING_RATE

            #Begrenzung der Geschwindigkeit
            linear_velocity = max(min(self.linear_velocity, linear_velocity), -self.linear_velocity)

            #Steuerbefehl senden 
            cmd = Twist()
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity
            self.cmd_pub.publish(cmd)

            self.get_logger().info(f"Steuerung: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, Marker-ID={marker_id}, Distanz={distance:.2f}")

        else:
            self.get_logger().warn("Kein Offset-Tupel verfügbar, Roboter bleibt stehen.")
            self.stop_robot()

    def stop_robot(self):
        #Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        #Kontrollschleife starten

        node = FollowerRobot()
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        node.get_logger().info("Folgen beendet (Tastaturunterbrechung).")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()