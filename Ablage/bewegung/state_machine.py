from enum import Enum


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
            return #aufrufen Node fahren
        elif self._state == _State.TURN:
            return #aufrufen Node drehen
        elif self._state == _State.FOLLOW:
            return #aufrufen Node folgen
        else: 
            return None