from enum import Enum
import time
from ar_pipe_server.drive import DriveNode



class _State(Enum):
    #mode
    IDLING = 0
    DRIVE = 1
    TURN = 2
    FOLLOW = 3


class ModeSelection:

    def __init__(self) -> None:
        self._state = _State.IDLING

    def cancel_target(self):
        self._state = _State.IDLING

    def select_mode(self, data_tuple, velocity):
        (id, _, _) = data_tuple
        id = int(id)
        if id == 2:
            self._state = _State.DRIVE
        if id == 1:
            self._state = _State.TURN
        if id == 0:
            self._state = _State.FOLLOW
        if id == 999:
            self._state = _State.IDLING
        
        if self._state == _State.IDLING:
            return None
        elif self._state == _State.DRIVE:
            return self.drive(data_tuple, velocity)
        elif self._state == _State.TURN:
            return self.turn(data_tuple)
        elif self._state == _State.FOLLOW:
            return self.follow(data_tuple)
        else: 
            return None
        
    def start_state(self):
        self._state = _State.IDLING
        
    def drive(self, data_tuple, velocity):
        print("driving with speed: " + str(velocity))
        self._state = _State.IDLING        
        return DriveNode().drive(data_tuple, velocity)

    def turn(self, data_tuple):
        print("turning")
        #time.sleep(1)
        self._state = _State.IDLING
        return None


    def follow(self, data_tuple):
        print("following")
        time.sleep(0.5)
        self._state = _State.IDLING
        return None