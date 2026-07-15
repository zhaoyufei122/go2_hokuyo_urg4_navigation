from datetime import datetime

from go2_base_nav.mapping_bag import (
    BAG_TOPICS,
    build_bag_record_command,
    select_bag_output,
)


EXPECTED_TOPICS = (
    "/utlidar/cloud_deskewed",
    "/utlidar/robot_odom",
    "/cloud_self_filtered",
    "/scan",
    "/map",
    "/tf",
    "/tf_static",
)


def test_select_bag_output_expands_root_and_avoids_existing_directory(tmp_path):
    now = datetime(2026, 7, 15, 12, 34, 56)
    first = tmp_path / "20260715_123456"
    first.mkdir()

    assert select_bag_output(tmp_path, now) == tmp_path / "20260715_123456_01"


def test_build_bag_record_command_contains_exact_topics_and_unique_output(
    tmp_path,
):
    now = datetime(2026, 7, 15, 12, 34, 56)

    command = build_bag_record_command(tmp_path, now)

    assert BAG_TOPICS == EXPECTED_TOPICS
    assert command == [
        "ros2",
        "bag",
        "record",
        "-o",
        str(tmp_path / "20260715_123456"),
        *EXPECTED_TOPICS,
    ]
