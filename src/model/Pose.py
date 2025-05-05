from src.model.custom_types.index import FootPositions, Position
from src.utils.ConfigHelper import ConfigHelper

class Pose:
    movement_config = ConfigHelper.get_section("movement_parameters")
    MAX_HEIGHT = movement_config.getfloat("max_height", fallback=15.0)
    MIN_HEIGHT = movement_config.getfloat("min_height", fallback=10.0)
    MID_HEIGHT = (MAX_HEIGHT + MIN_HEIGHT) / 2
    
    poses = {
        "STAND": FootPositions(
            FL=Position(x=0, y=-MAX_HEIGHT, z=0),
            FR=Position(x=0, y=-MAX_HEIGHT, z=0),
            BL=Position(x=0, y=-MAX_HEIGHT, z=0),
            BR=Position(x=0, y=-MAX_HEIGHT, z=0),
        ),
        "SIT": FootPositions(
            FL=Position(x=0, y=-MAX_HEIGHT, z=0),
            FR=Position(x=0, y=-MAX_HEIGHT, z=0),
            BL=Position(x=0, y=-MIN_HEIGHT, z=0),
            BR=Position(x=0, y=-MIN_HEIGHT, z=0),
        ),
        "PEE": FootPositions(
            FL=Position(x=0, y=-MID_HEIGHT, z=0),
            FR=Position(x=0, y=-MID_HEIGHT, z=0),
            BL=Position(x=0, y=-MID_HEIGHT, z=0),
            BR=Position(x=0, y=-MID_HEIGHT, z=5),
        ),
    }

    @staticmethod
    def get_pose_names() -> list[str]:
        return list(Pose.poses.keys())

    @staticmethod
    def get_pose_positions(pose_name: str) -> FootPositions | None:
        return Pose.poses.get(pose_name)