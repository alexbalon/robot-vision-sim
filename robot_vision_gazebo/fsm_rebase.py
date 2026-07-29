"""
fsm_rebase.py
Maquina de Estados Finitos (FSM) de evasion de obstaculos.
"""

from enum import Enum


class State(Enum):
    SEGUIR = 1
    DESVIAR = 2
    ADELANTAR = 3
    RETORNAR = 4
    PERDIDO = 5


class EvasionFSM:
    def __init__(self,
                 turn_duration: float = 1.5,
                 advance_duration: float = 1.2,
                 return_max_duration: float = 6.0,
                 turn_speed: float = 0.5,
                 linear_speed: float = 0.2):
        self.state = State.SEGUIR
        self.turn_duration = turn_duration
        self.advance_duration = advance_duration
        self.return_max_duration = return_max_duration
        self.turn_speed = turn_speed
        self.linear_speed = linear_speed
        self._elapsed_time = 0.0
        self.on_state_change = None

    def trigger(self, obstacle_detected: bool):
        if self.state == State.SEGUIR and obstacle_detected:
            self._enter_state(State.DESVIAR)

    def _enter_state(self, new_state):
        old_state = self.state
        self.state = new_state
        self._elapsed_time = 0.0
        if self.on_state_change is not None:
            self.on_state_change(old_state, new_state)

    def step(self, line_detected: bool = False, dt: float = 0.0):
        self._elapsed_time += dt

        if self.state == State.DESVIAR:
            if self._elapsed_time < self.turn_duration:
                return self.linear_speed, -self.turn_speed, True
            self._enter_state(State.ADELANTAR)

        if self.state == State.ADELANTAR:
            if self._elapsed_time < self.advance_duration:
                return self.linear_speed, 0.0, True
            self._enter_state(State.RETORNAR)

        if self.state == State.RETORNAR:
            if line_detected and self._elapsed_time >= 0.3:
                self._enter_state(State.SEGUIR)
                return 0.0, 0.0, False
            if self._elapsed_time < self.return_max_duration:
                # giro de busqueda mas lento y en el sitio (sin avance),
                # asi barre mas angulo por segundo sin alejarse mas del carril
                return self.linear_speed, self.turn_speed, True
            self._enter_state(State.PERDIDO)

        if self.state == State.PERDIDO:
            return 0.0, 0.0, False

        return 0.0, 0.0, False

    def is_following(self) -> bool:
        return self.state == State.SEGUIR
