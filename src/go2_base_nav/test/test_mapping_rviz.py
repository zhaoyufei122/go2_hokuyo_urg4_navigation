from pathlib import Path

import yaml


RVIZ_PATH = Path(__file__).resolve().parents[1] / "rviz" / "mapping.rviz"


def test_mapping_rviz_shows_filtered_cloud_scan_map_and_tf():
    config = yaml.safe_load(RVIZ_PATH.read_text())
    manager = config["Visualization Manager"]
    displays = {display["Name"]: display for display in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert displays["TF"]["Class"] == "rviz_default_plugins/TF"
    filtered_cloud = displays["Filtered Cloud"]
    assert filtered_cloud["Topic"]["Value"] == "/cloud_self_filtered"
    assert filtered_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert displays["Scan"]["Topic"]["Value"] == "/scan"
    assert displays["Scan"]["Topic"]["Reliability Policy"] == "Best Effort"
    assert displays["Map"]["Topic"]["Value"] == "/map"
    assert "Deskewed Cloud" not in displays
