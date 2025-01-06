import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

TURNING_RATE = 3.0      #Regelungsfaktor
MID_GOAL = 0.35         #Wert auf den geregelt wird

class DrivingRobot(Node):
    def __init__(self, user_speed):
        super().__init__('driving_robot')

        # ArUco-Marker-Daten abonnieren
        self.subscription_offset = self.create_subscription(
            String,
            'data',
            self.offset_callback,
            10
        )

        # Steuerbefehle publizieren
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialisiere Variablen
        self.offset_tuple = None  # Tuple (Marker-ID, Offset, Distanz)
        self.linear_velocity = user_speed  # Benutzerdefinierte Vorwärtsgeschwindigkeit
        self.kp_offset = 0.5  # Proportionalitätskonstante für Offset-Korrektur

    def offset_callback(self, msg):
        try:
            # Konvertiere ArUco-Daten aus String in ein Tupel
            data_tuple = tuple(map(float, msg.data.split()))
            self.offset_tuple = data_tuple
            #self.get_logger().info(f"Empfangenes Tupel: {self.offset_tuple}")
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Verarbeitung der Daten: {e}")

    def drive(self, data):
        if self.offset_tuple is not None:
            # Tupel extrahieren: (Marker-ID, Offset, Distanz)
            marker_id, offset, distance = self.offset_tuple

            # Offset-Korrektur (angular.z)
            offset = offset * -1.0
            rel_offset = offset / distance
            #print(str(rel_offset)            
            angular_velocity = MID_GOAL - rel_offset
            angular_velocity = angular_velocity * -TURNING_RATE

            if distance < 3.0:
                self.get_logger().info("Ziel erreicht")
                return self.stop_robot()
            
            
            
            # Steuerbefehl generieren
            cmd = Twist()
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = angular_velocity

            # Steuerbefehl senden
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Steuerung: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, Marker-ID={marker_id}, Distanz={distance:.2f}")
        else:
            self.get_logger().warn("Kein Offset-Tupel verfügbar, Roboter bleibt stehen.")
            self.stop_robot()

    def stop_robot(self):
        # Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    # Benutzerdefinierte Geschwindigkeit abfragen
    try:
        user_speed = float(input("Bitte geben Sie die Vorwärtsgeschwindigkeit des Roboters ein (m/s): "))
    except ValueError:
        print("Ungültige Eingabe. Standardgeschwindigkeit wird auf 0.1 m/s gesetzt.")
        user_speed = 0.1

    # Roboter mit benutzerdefinierter Geschwindigkeit starten
    node = DrivingRobot(user_speed)
    try:
        # Kontrollschleife starten
        while rclpy.ok():
            rclpy.spin_once(node)
            node.drive()
    except KeyboardInterrupt:
        node.get_logger().info("Fahren beendet (Tastaturunterbrechung).")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
