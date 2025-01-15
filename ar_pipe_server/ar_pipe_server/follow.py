from geometry_msgs.msg import Twist


TURNING_RATE = 3.0      #Regelungsfaktor
MID_GOAL = 0.35  

class FollowerNode:    

    offset_tuple = None #Tuple (Marker-ID, Offset, Distanz)
    linear_velocity = 0.2 #Maximale Vorwärtsgeschwindigkeit 
    kp_distance = 0.1 #Proportionalitätskonstante für Distanz
    kp_offset = 0.5 #Proportionalitätskonstante für Offset
    desired_distance = 2.0 #Soll-Abstand
    distance_threshold = 0.2 #Tolleranz für den Sollabstand
    #timer_period = 0.001
    #self.timer = create_timer(timer_period, self.offset_callback)        

    def follow_target(self, offset_tuple):
        if offset_tuple is not None:
            #Tupel extrahieren: (Marker-ID, Offset, Distanz)
            marker_id, offset, distance = offset_tuple
           


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
            

            #self.get_logger().info(f"Steuerung: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, Marker-ID={marker_id}, Distanz={distance:.2f}")
            
            #print(linear_velocity)
            #print(angular_velocity)
            return cmd, False, 3
        else:
            #self.get_logger().warn("Kein Offset-Tupel verfügbar, Roboter bleibt stehen.")
            self.stop_robot()

    def stop_robot(self):
        #Stoppt den Roboter
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd, True, 0
