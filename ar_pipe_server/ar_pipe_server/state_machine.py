from geometry_msgs.msg import Twist 
from enum import Enum
import time
from ar_pipe_server.drive import DriveNode
from ar_pipe_server.turn import TurningNode
from ar_pipe_server.follow import FollowerNode


class _State(Enum):
    #mode
    IDLING = 0
    DRIVE = 1
    SET_ANGLE = 2
    TURN = 3
    FOLLOW = 4


class ModeSelection:

    def __init__(self) -> None:
        self._state = _State.IDLING

    def cancel_target(self):
        self._state = _State.IDLING

    def select_mode(self, data_tuple, velocity, current_angle, dst_to_follow):            
        
        if self._state == _State.IDLING:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        elif self._state == _State.DRIVE:
            return self.drive(data_tuple, velocity)
        elif self._state == _State.TURN:
            return self.turn(current_angle)
        elif self._state == _State.FOLLOW:
            return self.follow(data_tuple, dst_to_follow)
        else: 
            return None
        
    def set_idling(self):
        self._state = _State.IDLING

    def set_drive(self):
        self._state = _State.DRIVE

    def set_turn(self):
        self._state = _State.TURN

    def set_follow(self):
        self._state = _State.FOLLOW    
    
    def set_target(self, theta):
        print("setting target")
        self.target_angle = TurningNode().set_target_angle(theta)
        print("target set " + str(self.target_angle))
        
    def drive(self, data_tuple, velocity):
        #print("driving with speed: " + str(velocity))
        #self._state = _State.IDLING        
        return DriveNode().drive(data_tuple, velocity)

    def turn(self, current_angle):
        #print("turning")
        #self._state = _State.IDLING
        return TurningNode().perform_turning(self.target_angle, current_angle)


    def follow(self, data_tuple, dst_to_follow):
        print("following")
        #self._state = _State.IDLING
        return FollowerNode().follow_target(data_tuple, dst_to_follow)