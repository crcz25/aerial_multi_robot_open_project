import yaml

from typing import Dict, List, Union, Tuple

class ConfigLaunch:
    def __init__(self, config_path: str) -> None:
        self.path = config_path
        self.config_dict = self.load_config_file(
            config_path=self.path
        )

    @staticmethod
    def load_config_file(config_path: str) -> Dict:
        with open(config_path, mode="rb") as file:
            config_dict = yaml.safe_load(file)
        return config_dict
    
    def get_robots_from_config(self) -> Union[List[str], None]:
        robots_config = self.config_dict.get('robots', None)

        robots_names, *robots_positions = (
            self.extract_robots_config(robots_config)
        )

        return robots_names, robots_positions

    def get_uwb_nodes_from_config(self) -> Union[List[str], None]:
        uwb_nodes_config = self.config_dict.get('uwb_nodes', None)

        uwb_nodes_names, uwb_nodes_positions = (
            self.extract_uwb_nodes_config(uwb_nodes_config)
        )
        return uwb_nodes_config, uwb_nodes_names, uwb_nodes_positions

    def get_uwb_ranges_from_config(self) -> Union[List[str], None]:
        uwb_ranges_names = self.config_dict.get('uwb_ranges', None)
        return uwb_ranges_names

    def extract_robots_config(
            self,
            robots_config: List[Union[Dict[str, Dict], str]]
    ) -> Tuple[List[str], List[float], List[float]]:
        robots_name = []
        robots_x_pos = []
        robots_y_pos = []

        # Variables to assign a pre-defined position in case the user does
        # not assign one.
        predef_robot_x_pos = 0
        predef_robot_y_pos = 0

        # Flag to ensure only one position type is allowed
        robots_position_type = 0

        for robot_config in robots_config:
            if (
                isinstance(robot_config, dict) and 
                (robots_position_type == 0 or robots_position_type == 1)
            ):
                robot_name, robot_pos = robot_config.popitem()
                robots_name.append(str(robot_name))

                robots_x_pos.append(robot_pos.get('x', 0))
                robots_y_pos.append(robot_pos.get('y', 0))

                robots_position_type = 1
            elif (
                isinstance(robot_config, str) and 
                (robots_position_type == 0 or robots_position_type == 2)
            ):
                robots_name.append(robot_config)

                robots_x_pos.append(predef_robot_x_pos)
                robots_y_pos.append(predef_robot_y_pos)

                predef_robot_x_pos += 1
                robots_position_type = 2
            else:
                raise ValueError("Inconsistent Robots' Position Definition")
        
        return robots_name, robots_x_pos, robots_y_pos
    
    def extract_uwb_nodes_config(
            self,
            uwb_nodes_in_config: Dict[str, Dict]
    ) -> Tuple[List[str], List[float], List[float]]:
        antennas_names = []
        nodes_antennas_positions = {}

        for node, config_node in uwb_nodes_in_config.items():

            node_ant_positions = {
                node: {
                    'x': [],
                    'y': [],
                    'z': []
                }
            }

            # Iterate over the antennas names
            robot_antennas_names = config_node['names']
            for antenna in robot_antennas_names:
                # Create the name of the antenna and save it
                antennas_names.append(f'{node}_{antenna}')
            
            try:
                # Try to get the list of antennas positions
                antennas_positions = config_node.get('positions', None)

                if antennas_positions is None:
                    # We put none as many antennas exist
                    antennas_none_pos = [None] * len(robot_antennas_names)
                    
                    node_ant_positions[node]['x'].extend(antennas_none_pos)
                    node_ant_positions[node]['y'].extend(antennas_none_pos)
                    node_ant_positions[node]['z'].extend(antennas_none_pos)
                else:
                    # Check if there are less positions than antennas
                    if len(antennas_positions) < len(robot_antennas_names):
                        raise ValueError(
                            f'Not all antennas for robot {node} have positions'
                        )

                    # If they exist, then go through each list of positions
                    # for each antenna, and store it in the corresponding place
                    # for the coordinates lists.
                    for idx, antenna_position in enumerate(antennas_positions):

                        # Check if the three coordinates exist
                        if len(antenna_position) < 3:
                            raise ValueError(
                                f'Antenna {antennas_names[idx]} is missing '
                                'a coordinate in its positions'
                            )

                        node_ant_positions[node]['x'].append(
                            antenna_position[0]
                        )
                        node_ant_positions[node]['y'].append(
                            antenna_position[1]
                        )
                        node_ant_positions[node]['z'].append(
                            antenna_position[2]
                        )

                nodes_antennas_positions.update(node_ant_positions)

            except IndexError as e:
                raise IndexError(
                    'Antennas are missing a coordinate or proper '
                    f'definition: {e}'
                )
            except ValueError as e:
                raise e
        
        return antennas_names, nodes_antennas_positions # antennas_x_pos, antennas_y_pos, antennas_z_pos


