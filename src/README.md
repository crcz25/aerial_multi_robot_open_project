Packages included in UWB Simulator Workspace

**Maybe?** *We could try to see how this works and if it does not fit appropriately then we can change for more or less packages.*

1. **robots_description**: includes the models and worlds for the possible scenarios that can be used in Gazebo for simulation with the UWB (drones and turtlebots / empty world, obstacles).
2. **uwb_simulator**: includes the Nodes to process all the UWB related data, as well as launch files to test the functioning of the system. It's necessary to fill a configuration file inside this package to accomodate the simulation to the required needs.

According to the configuration file requirements, we will launch a world including the robots mentioned, and simulate the behaviour of the UWB sensors with the given parameters. To launch the world we will search for the models in the robots_description, and will use the entity injection script available with the uwb_simulator package to create them in gazebo.