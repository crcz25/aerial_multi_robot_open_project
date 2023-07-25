# aerial_multi_robot_open_project
UWB Simulator

## To launch a simple world (from inside the main folder):

1.      colcon build
2.      source install/setup.bash
3. (To export the Turtlebot models- maybe this could be inside a sh file?) 
        
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/robots_description/share/robots_description/models
4.      export TURTLEBOT3_MODEL=waffle_pi
5.      ros2 launch uwb_simulator simple_world.launch.py


## Parameters to configure when launching the application:

- robots: They can be a list of names, or a list of each robot describing their position (see example).
- uwb_nodes: They define the same list of names as in the robots list, but they also define for each robot the number of antennas and the names that they will be asigned. Adding the position of each antenna (relative to the center of the robot) is optional.
- uwb_ranges: They configure how to measure the ranges between the antennas.

  - ground_truth: Name of the robot that will be used as ground truth. Meaning that all the other antennas will measure their distance against this robot but not between themselves. **This only applies if type_measurement is set to _origin_**
  - type_measurement: Determines the method to compute the ranges between the antennas.
    - "all": all the distances or ranges between all the pairs of antennas are computed.
    - "origin": the distances or ranges between all the antennas and the ground truth are computed (but not between the antennas that are not ground truth)
  - WIP

