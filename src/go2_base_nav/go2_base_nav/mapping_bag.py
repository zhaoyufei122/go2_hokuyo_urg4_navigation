from datetime import datetime
from pathlib import Path


BAG_TOPICS = (
    "/utlidar/cloud_deskewed",
    "/utlidar/robot_odom",
    "/cloud_self_filtered",
    "/scan",
    "/map",
    "/tf",
    "/tf_static",
)

SLAMTEC_BAG_TOPICS = (
    "/scan",
    "/utlidar/robot_odom",
    "/odom",
    "/map",
    "/map_updates",
    "/tf",
    "/tf_static",
)


def select_bag_output(
    root: str | Path,
    now: datetime | None = None,
) -> Path:
    root_path = Path(root).expanduser()
    root_path.mkdir(parents=True, exist_ok=True)
    stem = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    candidate = root_path / stem
    suffix = 1
    while candidate.exists():
        candidate = root_path / f"{stem}_{suffix:02d}"
        suffix += 1
    return candidate


def build_bag_record_command(
    root: str | Path,
    now: datetime | None = None,
) -> list[str]:
    output = select_bag_output(root, now)
    return ["ros2", "bag", "record", "-o", str(output), *BAG_TOPICS]


def build_slamtec_bag_record_command(
    root: str | Path,
    now: datetime | None = None,
) -> list[str]:
    output = select_bag_output(root, now)
    return [
        "ros2",
        "bag",
        "record",
        "-o",
        str(output),
        *SLAMTEC_BAG_TOPICS,
    ]
