From any terminal:

        ros2 run domain_bridge domain_bridge src/uwb_real_trajectories/domain_bridge_config/traj_bridge_tbot_config.yaml

This doesn't require to be in a specific ROS_DOMAIN_ID, however, in the configuration the global from and to domains seem to have problems, that's is why they are specified individually for each topic.