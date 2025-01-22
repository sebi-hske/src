from geometry_msgs.msg import Twist 
from tf_transformations import euler_from_quaternion
import math
import time

ANGLE_THRESHOLD = 0.04
ANGULAR_VELOCITY = 0.3 

class TurningNode:
    
    def set_target_angle(self, theta):
        print("orientation set " + str(theta))        
        #Setze den Zielwinkel für die Drehung (180° Wende)
        #target_angle = self.normalize_angle(theta + math.pi)
        if theta < 0.0:
            target_angle = theta + math.pi
        elif theta > 0.0:
            target_angle = theta - math.pi
        return target_angle
        

    def perform_turning(self, target_angle, current_angle):        
        print("perform turn")        
        cmd = Twist()
        cmd.linear.x = 0.0 #Keine Vorwärtsbewegung während des Wendens
        #Berechne die Abweichung zum Zielwinkel
        if target_angle < 0.0: 
            target_angle = target_angle * -1.0   
            angular_error = target_angle - current_angle
        elif target_angle > 0.0:
            angular_error = target_angle - current_angle
        #Prüfen, ob die Drehung abgeschlossen ist
        if angular_error < 0.0:
            angular_error = angular_error * -1.0
        print("abweichung " + str(angular_error))
        #if angular_error > 3.2:
        #    cmd.angular.z = 0.0
        #    return cmd, True, 2
        if angular_error < ANGLE_THRESHOLD:
            print("zielwinkel erreicht")
            cmd.angular.z = 0.0

            return cmd, True, 2
        else: 
            cmd.angular.z = ANGULAR_VELOCITY
            return cmd, False, 1
            ##angular_error = self.normalize_angle(angular_error)
            #if angular_error > 0:
            #    cmd.angular.z = ANGULAR_VELOCITY
            #    return cmd
            #else: 
            #    cmd.angular.z = -ANGULAR_VELOCITY
            #    return cmd

    def stop_robot(self):
        #Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd, False, 0

    def normalize_angle(self, angle):
        #Normalisiert einen Winkel in den Bereich [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle