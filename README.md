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
  - Option 1:
  -     - T01:
          x: 1.0
          y: 1.5
        - T02:
          x: 2.0
          y: 1.5
  - Option 2:
  -     - T01
        - T02
- uwb_nodes: They define the same list of names as in the robots list, but they also define for each robot the number of antennas and the names that they will be asigned. Adding the position of each antenna (relative to the center of the robot) is optional. It is mandatory to include the information for all the robots declared in the **robots** section.
  - Example:   
  -     T01:
          num_antennas: 1
          names: [
            "A",
          ]
          positions: [
            [0.5, 0.5, 0],
          ]
- uwb_ranges: They configure how to measure the ranges between the antennas.

  - ground_truth: Name of the robot that will be used as ground truth. Meaning that all the other antennas will measure their distance against this robot but not between themselves. **This only applies if type_measurement is set to _origin_**
  - type_measurement: Determines the method to compute the ranges between the antennas. The distances between the antennas of the same robot are never computed
    - "all": all the distances or ranges between all the pairs of antennas are computed.
    - "origin": the distances or ranges between all the antennas and the ground truth are computed (but not between the antennas that are not ground truth)
  - localization_method: Determines the method to compute the absolute or relative positions of the robot based on the ranges measured in the simulator. These estimated positions are saved in a external file.
    - "lse": Uses a least squares error estimator to compute the absolute positions of the robots.
    - Any other string: Disables this option and no estimation is done.
  - write_to_file: A flag that allows writing a file containing key information from the simulator. It includes the ranges as columns, absolute positions (or real positions) as well as estimated positions if the a method is selected
  - max_twr_freq: Frequency at which the simulator computes the ranges for each pair of antennas (every X Hz a new measurement is computed and published)
  - duty_cycle: ?
  - mean: Mean of the Gaussian distribution modelling of the ranges measurements noise (normally the value is zero)
  - std_dev: Standard deviation of the Gaussian distribution modelling of the ranges measurements noise.
  - ranges: A list of the robots that are not ground truth robots, the ranges measurements will be computed for this robots (between these robots and the ground truth robot defined before).

- demo_trajectory: A flag that activates the movement of robots in a pre-defined trajectory.

