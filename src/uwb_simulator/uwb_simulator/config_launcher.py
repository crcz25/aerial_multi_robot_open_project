import yaml

from typing import Dict, List, Union

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
        robots_names = self.config_dict.get('robots', None)
        return robots_names

    def get_uwb_nodes_from_config(self) -> Union[List[str], None]:
        uwb_nodes_names = self.config_dict.get('uwb_nodes', None)
        return uwb_nodes_names

    def get_uwb_ranges_from_config(self) -> Union[List[str], None]:
        uwb_ranges_names = self.config_dict.get('uwb_ranges', None)
        return uwb_ranges_names
