import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


TURNING_RATE = 3.0      #Regelungsfaktor
MID_GOAL = 0.35         #Wert auf den geregelt wird


class DriveNode:

    def drive(self, data, velocity):
        if data is not None:
            # Tupel extrahieren: (Marker-ID, Offset, Distanz)
            marker_id, offset, distance = data

            # Offset-Korrektur (angular.z)
            offset = offset * -1.0
            rel_offset = offset / distance
            #print(str(rel_offset)            
            angular_velocity = MID_GOAL - rel_offset
            angular_velocity = angular_velocity * -TURNING_RATE

            if distance < 3.0:
                return self.stop_robot()
            
            
            
            # Steuerbefehl generieren
            cmd = Twist()
            cmd.linear.x = velocity
            cmd.angular.z = angular_velocity

            # Steuerbefehl senden
            return cmd
        else:
            self.stop_robot()

    def stop_robot(self):
        # Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd