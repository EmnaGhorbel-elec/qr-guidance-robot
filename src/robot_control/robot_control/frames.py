"""Changements de repere monde <-> odom (fonctions pures, sans ROS).

Le repere `odom` est celui de l'odometrie des roues : il demarre a (0, 0, 0) a la position
de depart du robot et DERIVE avec le temps. Les positions des QR codes sont connues dans le
repere du monde. Quand la localisation par QR donne la pose du robot dans le monde, on en
deduit la transformation monde <-> odom, puis on convertit les objectifs.

Poses : (x, y, yaw) en metres / radians. Points : (x, y).

Exemple :
    tf = estimate_world_to_odom(pose_monde_depuis_qr, pose_odom_au_meme_instant)
    gx, gy = world_to_odom((x_qr, y_qr), tf)       # a publier sur /goal
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def estimate_world_to_odom(world_pose, odom_pose):
    """Pose du repere odom dans le monde : (tx, ty, dyaw), a partir de la pose du robot
    dans le monde (QR) et de sa pose dans odom, mesurees au MEME instant."""
    xw, yw, thw = world_pose
    xo, yo, tho = odom_pose
    dth = wrap(thw - tho)
    c, s = math.cos(dth), math.sin(dth)
    return (xw - (c * xo - s * yo), yw - (s * xo + c * yo), dth)


def odom_to_world(point_odom, transform):
    tx, ty, dth = transform
    c, s = math.cos(dth), math.sin(dth)
    return (c * point_odom[0] - s * point_odom[1] + tx,
            s * point_odom[0] + c * point_odom[1] + ty)


def world_to_odom(point_world, transform):
    tx, ty, dth = transform
    px, py = point_world[0] - tx, point_world[1] - ty
    c, s = math.cos(-dth), math.sin(-dth)
    return (c * px - s * py, s * px + c * py)
