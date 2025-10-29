import math

def twist_to_wheels_rpm(v_linear, w_angular, wheel_radius_m, track_width_m):
    """
    Convierte velocidades lineal (v) y angular (w) del robot
    en velocidades de ruedas (izquierda y derecha) expresadas en RPM.

    v_linear: velocidad lineal [m/s]
    w_angular: velocidad angular [rad/s]
    wheel_radius_m: radio de la rueda [m]
    track_width_m: distancia entre ruedas [m]
    """
    # Cinemática diferencial
    v_r = v_linear + (w_angular * track_width_m / 2.0)
    v_l = v_linear - (w_angular * track_width_m / 2.0)

    # Convertir de m/s a RPM
    rpm_r = (v_r / (2 * math.pi * wheel_radius_m)) * 60.0
    rpm_l = (v_l / (2 * math.pi * wheel_radius_m)) * 60.0

    return rpm_l, rpm_r
