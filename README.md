# Metody i algorytmy planowania ruchu robotów - Projekt
## Uruchomienie symulatora
```console
foo@bar:~/ros2_miapr$ ./src/Universal_Robots_ROS2_Driver/ur_robot_driver/scripts/start_ursim.sh -m ur5e
```
## Uruchomienie symulatora robota
```console
foo@bar:~/ros2_miapr$ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```
## Uruchomienie sterownika
```console
foo@bar:~/ros2_miapr$ ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true
```