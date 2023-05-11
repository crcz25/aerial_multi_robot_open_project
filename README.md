# aerial_multi_robot_open_project
UWB Simulator

To launch a simple world (from inside the main folder):

1.      colcon build
2.      source install/setup.bash
3. (To export the Turtlebot models- maybe this could be inside a sh file?) 
        
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/robots_description/share/robots_description/models
4.      ros2 launch uwb_simulator simple_world.launch.py

