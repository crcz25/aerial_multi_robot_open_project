To execute the trajectories you need to issue the following command in a terminal with the same ROS_DOMAIN_ID. The ROS_DOMAIN_ID is the number that identifies the network in which the ROS nodes are going to communicate. If you are using the default configuration, you can skip this step.

```bash
ros2 run uwb_real_trajectories tbot_trajectories --ros-args -p n_loop:=num_loops -p trajectory:=type_trajectory -p sim:=bool
ros2 run uwb_real_trajectories tello_trajectories --ros-args -p n_loop:=num_loops -p trajectory:=type_trajectory -p sim:=bool
```

where:
 - num_loops: number of times the trajectory is going to be executed.
 - trajectory: string containing the trajectory to be followed (circle or square).
 - bool: boolean indicating if the trajectory is going to be executed in simulation or in the real robot (True or False).

