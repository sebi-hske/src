# pd_regler.py

class PDRegler:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.fehler_alt = 0.0

    def berechne(self, sollwert, istwert, dt):
        fehler = sollwert - istwert
        d_fehler = (fehler - self.fehler_alt) / dt if dt > 0 else 0.0
        self.fehler_alt = fehler
        return self.kp * fehler + self.kd * d_fehler
