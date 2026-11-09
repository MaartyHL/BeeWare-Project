"""Classe PID réutilisable pour le contrôle de vol BeeWare.

- clamp de la sortie (out_min / out_max)
- anti-windup : clamp de l'intégrale (i_max)
- dérivée filtrée (d_alpha) ; on peut passer la dérivée de la mesure
  (derivative=...) pour éviter le « derivative kick » quand la consigne change.
"""


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, out_min=-1e9, out_max=1e9, i_max=1e9, d_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max = i_max
        self.d_alpha = d_alpha  # 1.0 = pas de filtrage de la dérivée
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0
        self.has_prev = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0
        self.has_prev = False

    def compute(self, error, dt, derivative=None):
        """Retourne la commande pour l'erreur donnée.

        dt : pas de temps en secondes.
        derivative : d(erreur)/dt si on la connaît (ex : -d(mesure)/dt quand la
        consigne est constante) ; sinon la dérivée est calculée sur l'erreur.
        """
        if dt <= 0.0:
            dt = 1e-3
        p = self.kp * error
        self.integral = max(-self.i_max, min(self.i_max, self.integral + error * dt))
        if derivative is not None:
            d_raw = derivative
        elif self.has_prev:
            d_raw = (error - self.prev_error) / dt
        else:
            d_raw = 0.0
        d_filt = self.d_alpha * d_raw + (1.0 - self.d_alpha) * self.prev_deriv
        self.prev_deriv = d_filt
        self.prev_error = error
        self.has_prev = True
        out = p + self.ki * self.integral + self.kd * d_filt
        return max(self.out_min, min(self.out_max, out))
