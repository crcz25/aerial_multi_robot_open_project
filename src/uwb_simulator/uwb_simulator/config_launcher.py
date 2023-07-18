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
        uwb_nodes_names = self.config_dict.get('uwb_nodes', None)
        return uwb_nodes_names

    def get_uwb_ranges_from_config(self) -> Union[List[str], None]:
        uwb_ranges_names = self.config_dict.get('uwb_ranges', None)
        return uwb_ranges_names

    def extract_robots_config(
            self,
            robots_config: List[Union[Dict[str, Dict], str]]
    ) -> Tuple[List[str], List[int], List[int]]:
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


