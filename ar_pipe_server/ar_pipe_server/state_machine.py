from enum import Enum
from ar_pipe_server import drive_node
from ar_pipe_server import turn_node
from ar_pipe_server import follow_node


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
            return drive_node()
        elif self._state == _State.TURN:
            return turn_node()
        elif self._state == _State.FOLLOW:
            return follow_node()
        else: 
            return None