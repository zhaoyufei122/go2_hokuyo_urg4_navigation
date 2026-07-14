from unittest.mock import Mock

import pytest

from go2_base_nav import cmd_vel_bridge, planar_odom


@pytest.mark.parametrize(
    ("module", "node_class_name"),
    (
        (planar_odom, "PlanarOdomNode"),
        (cmd_vel_bridge, "Go2CmdVelBridge"),
    ),
)
def test_main_does_not_shutdown_an_already_stopped_context(
    monkeypatch,
    module,
    node_class_name,
):
    node = Mock()
    shutdown = Mock()

    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "spin_once", Mock())
    monkeypatch.setattr(module.rclpy, "ok", Mock(return_value=False))
    monkeypatch.setattr(module.rclpy, "shutdown", shutdown)

    module.main()

    node.destroy_node.assert_called_once_with()
    shutdown.assert_not_called()


@pytest.mark.parametrize(
    ("module", "node_class_name"),
    (
        (planar_odom, "PlanarOdomNode"),
        (cmd_vel_bridge, "Go2CmdVelBridge"),
    ),
)
def test_main_shuts_down_a_running_context(
    monkeypatch,
    module,
    node_class_name,
):
    node = Mock()
    shutdown = Mock()

    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "spin_once", Mock())
    monkeypatch.setattr(module.rclpy, "ok", Mock(side_effect=(True, False, True)))
    monkeypatch.setattr(module.rclpy, "shutdown", shutdown)

    module.main()

    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
