"""Fonctions pures (sans ROS) : faciles a tester seules."""
import math


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_angle(a):
    """Ramene un angle dans [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def ramp(current, target, max_step):
    """Rapproche current de target, au plus de max_step (limite d'acceleration)."""
    return current + clip(target - current, -max_step, max_step)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def go_to_point(x, y, yaw, gx, gy, k_lin, k_ang, max_v, max_w, tol, turn_thresh):
    """Controleur proportionnel vers un point (gx, gy) du repere odom.

    Retourne (v, w, reached).
    - si l'erreur de cap est grande : le robot tourne sur place
    - sinon il avance, avec une vitesse proportionnelle a la distance
    """
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist < tol:
        return 0.0, 0.0, True
    err = wrap_angle(math.atan2(dy, dx) - yaw)
    if abs(err) > turn_thresh:
        v = 0.0
    else:
        v = min(k_lin * dist, max_v) * max(0.0, math.cos(err))
    w = clip(k_ang * err, -max_w, max_w)
    return float(v), float(w), False
