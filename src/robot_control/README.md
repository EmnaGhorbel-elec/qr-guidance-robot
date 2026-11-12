# robot_control — nœuds de commande et moteur

Paquet ROS 2 Humble (Python) pour le robot mobile 4 roues (projet « QR code + localisation + guidage »).
Il contient deux nœuds séparés :

- **`command_node`** : décide. Il reçoit un objectif (x, y) et, avec le retour `/odom`, calcule la vitesse à envoyer.
- **`motor_node`** : interface moteur. Il limite les vitesses, lisse les accélérations et arrête le robot si la commande disparaît (watchdog).

## Architecture

```
/goal (Point) ──► command_node ──/cmd_vel_raw──► motor_node ──/cmd_vel──► ros_gz_bridge ──► Gazebo (DiffDrive)
                       ▲                                                                          │
                       └──────────────────────────────── /odom ◄──────────────────────────────────┘
```

Avec de vrais moteurs, seul `motor_node` change : il enverrait les vitesses de roues (`/wheel_speeds`) au driver
et publierait `/odom` à partir des encodeurs. `command_node` reste identique.

## Interfaces

| Nœud | Topic | Type | Sens | Description |
|---|---|---|---|---|
| command_node | `/odom` | `nav_msgs/Odometry` | entrée | Position et cap du robot (feedback) |
| command_node | `/goal` | `geometry_msgs/Point` | entrée | Objectif (x, y) en mètres, repère `odom` (z ignoré) |
| command_node | `/stop` | `std_msgs/Empty` | entrée | Annule l'objectif en cours |
| command_node | `/cmd_vel_raw` | `geometry_msgs/Twist` | sortie | Vitesse demandée (linear.x en m/s, angular.z en rad/s) |
| command_node | `/goal_reached` | `std_msgs/Bool` | sortie | `False` à l'envoi d'un objectif, `True` quand il est atteint |
| motor_node | `/cmd_vel_raw` | `geometry_msgs/Twist` | entrée | Vitesse demandée |
| motor_node | `/cmd_vel` | `geometry_msgs/Twist` | sortie | Vitesse limitée et lissée, lue par Gazebo via le pont |
| motor_node | `/wheel_speeds` | `std_msgs/Float64MultiArray` | sortie | [gauche, droite] en rad/s (informatif) |

## Paramètres

`command_node` : `k_lin` 0.8 · `k_ang` 1.5 · `max_v` 0.2 m/s · `max_w` 0.8 rad/s · `tolerance` 0.05 m · `turn_threshold` 0.5 rad · `rate_hz` 20

`motor_node` : `max_linear` 0.3 m/s · `max_angular` 1.5 rad/s · `max_linear_acc` 0.5 m/s² · `max_angular_acc` 2.0 rad/s² · `cmd_timeout` 0.8 s · `rate_hz` 50 · `wheel_radius` 0.033 m · `wheel_separation` 0.147 m

Réglage à chaud : `ros2 param set /command_node max_v 0.1`

## Lancement

Prérequis : Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (`ros-humble-ros-gz`), URDF `URDFF_gazebo.urdf`, monde Gazebo de la classe.

```bash
cd <workspace>
colcon build --symlink-install --packages-select robot_control
source install/setup.bash

# simulation (monde + robot + pont /cmd_vel /odom /tf /clock), puis :
ros2 launch robot_control control.launch.py

# envoyer un objectif
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"
```

## Conventions et limites

- Le repère `odom` a pour origine la position de départ du robot à chaque lancement de la simulation (odométrie des roues : elle dérive avec le temps).
- Dans l'URDF, la caméra et les roues avant sont du côté **-x** de `base_link`, et la caméra regarde vers -x. Le sens réel obtenu avec `linear.x > 0` est à vérifier au premier lancement (procédure dans `LISEZ-MOI.md` à la racine du dossier d'intégration).
- Un seul objectif à la fois. Pas d'évitement d'obstacles dans ce paquet (le LiDAR n'est pas utilisé ici).
- Quand `motor_node` tourne, ne pas publier directement sur `/cmd_vel` : utiliser `/cmd_vel_raw`.
- Watchdog : sans message sur `/cmd_vel_raw` pendant 0,8 s, le robot décélère jusqu'à l'arrêt.

## Intégration avec les autres nœuds

Le nœud de guidage par QR code (ou de localisation) n'a qu'une chose à faire : publier le prochain point à atteindre sur `/goal`
(`geometry_msgs/Point`, repère `odom`) et surveiller `/goal_reached`.

## Changements de repère (monde <-> odom)

`robot_control.frames` fournit `estimate_world_to_odom`, `world_to_odom` et `odom_to_world` pour convertir
des positions connues dans le monde (QR codes) en objectifs dans le repère `odom`. Voir `LISEZ-MOI.md`.
