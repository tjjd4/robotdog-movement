import configparser
from configparser import SectionProxy
from typing import Any, List, Dict

class ConfigHelper:
    _config = None
    _config_path = "config.ini"

    @classmethod
    def load_config(cls) -> None:
        if cls._config is None:
            cls._config = configparser.ConfigParser()
            cls._config.read(cls._config_path)

    @classmethod
    def set_config_path(cls, filepath: str) -> None:
        cls._config_path = filepath
        cls._config = None  # Reset to ensure the new config file is loaded

    @classmethod
    def get_section(cls, section: str) -> SectionProxy:
        cls.load_config()
        if section not in cls._config:
            raise ValueError(f"Section '{section}' not found in the configuration file.")
        return cls._config[section]

if __name__ == "__main__":
    ConfigHelper.set_config_path("config.ini")

    robotdog_parameters = ConfigHelper.get_section("robotdog_parameters")
    legs_motors = ConfigHelper.get_section("legs_motors")
    controller_network = ConfigHelper.get_section("controller_network")

    print("Robotdog Parameters Config:", robotdog_parameters)
    print("Controller Network Config:", controller_network)

    FL_opposited = legs_motors.getboolean("FL_is_opposited")
    print("FL Opposited: ", FL_opposited)
