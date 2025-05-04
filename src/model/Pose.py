

from src.model.custom_types.index import FootPositions, Position

class Pose:
    poses = {
        "STAND": FootPositions(
            FL=Position(x=0, y=-15, z=0),
            FR=Position(x=0, y=-15, z=0),
            BL=Position(x=0, y=-15, z=0),
            BR=Position(x=0, y=-15, z=0),
        ),
        "SIT": FootPositions(
            FL=Position(x=5, y=-15, z=5),
            FR=Position(x=5, y=-15, z=-5),
            BL=Position(x=-5, y=-15, z=5),
            BR=Position(x=-5, y=-15, z=-5),
        ),
        "PEE": FootPositions(
            FL=Position(x=5, y=-20, z=5),
            FR=Position(x=5, y=-20, z=-5),
            BL=Position(x=-5, y=-20, z=5),
            BR=Position(x=-5, y=-20, z=-5),
        ),
    }

    @staticmethod
    def get_pose_names() -> list[str]:
        return list(Pose.poses.keys())

    @staticmethod
    def get_pose_positions(pose_name: str) -> FootPositions | None:
        return Pose.poses.get(pose_name)