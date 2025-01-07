from geometry_msgs.msg import Twist 
from tf_transformations import euler_from_quaternion
import math
import time

ANGLE_THRESHOLD = 0.02
ANGULAR_VELOCITY = 0.4

class TurningNode:
    
    def set_target_angle(self, theta):
        print("orientation set " + str(theta))        
        #Setze den Zielwinkel für die Drehung (180° Wende)
        #target_angle = self.normalize_angle(theta + math.pi)
        target_angle = theta + math.pi
        return target_angle
        

    def perform_turning(self, target_angle, current_angle):        
        print("executing")        
        cmd = Twist()
        cmd.linear.x = 0.0 #Keine Vorwärtsbewegung während des Wendens
        #Berechne die Abweichung zum Zielwinkel        
        angular_error = target_angle - current_angle
        print("abwichung " + str(angular_error))
        #Prüfen, ob die Drehung abgeschlossen ist
        if abs(angular_error) <= ANGLE_THRESHOLD:
            print("zielwinkel erreicht")
            cmd.angular.z = 0.0
            return cmd
        else: 
            angular_error = self.normalize_angle(angular_error)
            if angular_error > 0:
                cmd.angular.z = ANGULAR_VELOCITY
                return cmd
            else: 
                cmd.angular.z = -ANGULAR_VELOCITY
                return cmd

    def stop_robot(self):
        #Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    def normalize_angle(self, angle):
        #Normalisiert einen Winkel in den Bereich [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle