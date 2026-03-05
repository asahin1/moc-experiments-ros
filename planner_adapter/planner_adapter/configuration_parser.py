import ast
import json

from hitch_planning.tether.configuration import TetherConfiguration, ConfigItem


def read_csv_and_parse(filepath):
    result = {}
    if filepath.exists():
        with open(filepath, "r") as f:
            for i, line in enumerate(f):
                line_contents = json.loads(line)
                result[i] = []
                for tether_content in line_contents:
                    tether_config = []
                    for item in tether_content:
                        tether_config.append(ConfigItem(item[0], item[1]))
                    result[i].append(TetherConfiguration(tether_config))
    else:
        raise ValueError(f"Result path {filepath} does not exist.")

    return result


class ConfigurationParser:
    def __init__(self, inputPath):
        self.inputPath = inputPath
        self.configDict = read_csv_and_parse(self.inputPath)
