import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

class TurningRobot(Node):
    def __init__(self):
        super().__init__('turning_robot')

        #Odometrie abonnieren
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        #Steuerbefehl publizieren
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #Initialisiere Variablen
        self.target_angle = None #Zielwinkel für das Wenden
        self.current_angle = None #Aktueller Winkel des Roboters
        self.angular_velocity = 0.5 #Drehgeschwindigkeit
        self.angle_threshold = 0.05 #Schwellenwert fpr den Zielwinkel (in Radiant)
        self.turning_complete = False #Status für abgeschlossenes Wenden

    def odom_callback(self,msg):
        #Extrahiere die aktuelle Orientierung aus der Odometrie
        orientation = msg.pose.pose.orientation
        _, _, self.current_angle = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        #self.get_logger().info(f"Aktueller Winkel: {math.degrees(self.current_angle):.2f}°")

    def start_turning(self):
        print("start turn")
        if self.current_angle is None:
            self.get_logger().warn("Kein aktueller Winkel verfügbar, Wenden wird abgebrochen.")
            return 
        
        #Setze den Zielwinkel für die Drehung (180° Wende)
        self.target_angle = self.normalize_angle(self.current_angle + math.pi)
        self.get_logger().info(f"Zielwinkel gesetzt: {math.degrees(self.target_angle):.2f}°")

    def perform_turning(self):
        if self.target_angle is None or self.current_angle is None:
            self.get_logger().warn("Zielwinkel oder aktueller Winkel fehlt, Roboter bleibt stehen.")
            self.stop_robot()

        print("executing")
        
        #Berechne die Abweichung zum Zielwinkel
        while rclpy.ok():
            angular_error = self.normalize_angle(self.target_angle - self.current_angle)

            #Prüfen, ob die Drehung abgeschlossen ist
            if abs(angular_error) < self.angle_threshold:
                self.get_logger().info("Wenden abgeschlossen.")
                self.stop_robot()
                self.turning_complete = True
                break

            #Steuerbefehl für die Drehung
            cmd = Twist()
            cmd.linear.x = 0.0 #Keine Vorwärtsbewegung während des Wendens
            cmd.angular.z = self.angular_velocity if angular_error > 0 else -self.angular_velocity
            self.cmd_pub.publish(cmd)
            
            self.get_logger().info(f"Drehe: angular.z={cmd.angular.z:.2f}, Fehler={math.degrees(angular_error):.2f}°")

    def stop_robot(self):
        #Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        #Normalisiert einen Winkel in den Bereich [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
def main(args=None):
    rclpy.init(args=args)
    node = TurningRobot()
    try:
        #Starte das Wenden
        node.start_turning()
        while rclpy.ok():
            rclpy.spin_once(node)
            node.perform_turning()
    except KeyboardInterrupt:
        node.get_logger().info("Wenden beendet (Tastaturunterbrechung).")
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()