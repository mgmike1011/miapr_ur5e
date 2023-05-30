# Metody i algorytmy planowania ruchu robotów - Projekt - 2023
# Agnieszka Piórkowska, Miłosz Gajewski
## Uruchomienie symulatora
```console
foo@bar:~/ros2_miapr$ ./src/Universal_Robots_ROS2_Driver/ur_robot_driver/scripts/start_ursim.sh -m ur5e
```
## Uruchomienie symulatora robota
```console
foo@bar:~/ros2_miapr$ ros2 launch ur_robot_driver ur_control.launch.py \ 
ur_type:=ur5e robot_ip:=192.168.56.101 \
use_fake_hardware:=true launch_rviz:=false \
initial_joint_controller:=joint_trajectory_controller
```
## Uruchomienie sterownika
```console
foo@bar:~/ros2_miapr$ ros2 launch ur_moveit_config \
ur_moveit.launch.py ur_type:=ur5e \
launch_rviz:=true use_fake_hardware:=true
```
## Zadanie:
Implementacja modułu do planowania ruchu manipulatora UR5 w ROS 2 (Humble) z wykorzystaniem MoveIt.
- uruchomienie symulacji robota w Gazebo+Rviz;
- implementacja węzła w C++ do planowania ruchu manipulatora - trajektoria do celu wyrażonego w joint space oraz cartesian space, trajektoria liniowa w cartesian space;
- wykorzystać należy MoveIt (kinematyka, planning scene), do utworzenia ścieżki - move_group,  umożliwienie wyboru planera z poziomu kodu, porównanie kilku algorytmów planowania;   
- planowanie ścieżki z unikaniem kolizji ze środowiskiem reprezentowanym jako Octomapa;

## Uruchomienie servera
```console
foo@bar:~/ros2_miapr$ ros2 launch miapr_ur5e trajectory_control_server.launch.py
``` 

## Trajectory controllers
### Joint Space Trajectory
```console
foo@bar:~/ros2_miapr$ ros2 service call /joint_trajectory_service miapr_ur5e_interfaces/srv/JointTrajectoryInterface "{j1: 0, j2: -1.57, j3: -1.57, j4: -1.57, j5: 1.57, j6: 0, controller: 1}"
```

### Cartesian Space Trajectory
```console
foo@bar:~/ros2_miapr$ ros2 service call /cartesian_trajectory_service miapr_ur5e_interfaces/srv/CartesianTrajectoryInterface "{x: 0.2, y: 0, z: 0, qw: 0, qx: 0, qy: 0, qz: 0, controller: 1}"
```

### Cartesian Linear Space Trajectory
```console
foo@bar:~/ros2_miapr$ ros2 service call /cartesian_linear_trajectory_service miapr_ur5e_interfaces/srv/CartesianTrajectoryInterface "{x: 0.2, y: 0, z: 0, qw: 0, qx: 0, qy: 0, qz: 0}"
```
## Przeszkody

### Dodanie przeszkody do sceny
```console
foo@bar:~/ros2_miapr$ ros2 service call /obstacle_add_service miapr_ur5e_interfaces/srv/ObstacjeInterface "{x: 1, y: 1, z: -2, box_x: 1, box_y: 1, box_z: 1}"
```

### Usuwanie przeszkód ze sceny
```console
foo@bar:~/ros2_miapr$ ros2 service call /obstacle_del_service miapr_ur5e_interfaces/srv/ObstacleDelInterface
```

## Interfejsy dodatkowe
### CartesianTrajectoryInterface.srv
```console
# 
# Request: position, Response: status
# Controlers: 0 - SBLkConfigDefault, 1 - ESTkConfigDefault, 2 - LBKPIECEkConfigDefault, 3 - BKPIECEkConfigDefault, 4 - KPIECEkConfigDefault, 5 - RRTkConfigDefault
# 6 - RRTConnectkConfigDefault, 7 - RRTstarkConfigDefault, 8 - TRRTkConfigDefault, 9 - PRMkConfigDefault, 10 - PRMstarkConfigDefault
# 
float64 x
float64 y
float64 z
float64 qw
float64 qx
float64 qy
float64 qz
uint8 controller
---
bool status
```
### JointTrajectoryInterface.srv
```console
# 
# Request: joiny values, Response: status
# Controlers: 0 - SBLkConfigDefault, 1 - ESTkConfigDefault, 2 - LBKPIECEkConfigDefault, 3 - BKPIECEkConfigDefault, 4 - KPIECEkConfigDefault, 5 - RRTkConfigDefault
# 6 - RRTConnectkConfigDefault, 7 - RRTstarkConfigDefault, 8 - TRRTkConfigDefault, 9 - PRMkConfigDefault, 10 - PRMstarkConfigDefault
# 
float64 j1
float64 j2
float64 j3
float64 j4
float64 j5
float64 j6
uint8 controller
---
bool status
```
### ObstacjeInterface.srv
```console
# 
# Position: x,y,z Dimensions: box_x, box_y, box_z
# 
float64 x
float64 y
float64 z
float64 box_x
float64 box_y
float64 box_z
---
bool status
```
### ObstacleDelInterface.srv
```console
# 
# Clear the stage
# 
bool delete_all
---
bool status
```