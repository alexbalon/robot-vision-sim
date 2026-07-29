"""
pid_controller.py
Regulador PD/PID de velocidad angular para el seguimiento de línea.
Entrada:  error (px) = Cx (centroide detectado) - Setpoint (centro de imagen)
Salida:   angular_z (rad/s) para el tópico /cmd_vel
"""


class PIDController:
    def __init__(self, kp: float = 0.005, kd: float = 0.002, ki: float = 0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, error: float, dt: float = 0.0) -> float:
        """Calcula la salida angular_z a partir del error actual y el tiempo transcurrido."""
        p = error
        d = 0.0 if dt <= 0.0 else (error - self.last_error) / dt
        self.integral += error * dt

        output = (self.kp * p) + (self.kd * d) + (self.ki * self.integral)

        self.last_error = error
        return output

    def reset(self):
        """Reinicia el estado interno (usar al entrar en modo evasión)."""
        self.last_error = 0.0
        self.integral = 0.0

    def update_gains(self, kp=None, kd=None, ki=None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if ki is not None:
            self.ki = ki
