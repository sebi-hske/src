from enum import Enum
import time


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

    def select_mode(self):
        if self._state == _State.IDLING:
            return None
        elif self._state == _State.DRIVE:
            return self.drive()
        elif self._state == _State.TURN:
            return self.turn()
        elif self._state == _State.FOLLOW:
            return self.follow()
        else: 
            return None
        
    def drive(self, data_tuple):

        print("driving with input: " + data_tuple)
        time.sleep(2)
        self._state = _State.TURN

    def turn(self):
        print("turning")
        time.sleep(2)
        self._state = _State.DRIVE

    def follow(self):
        print("following")