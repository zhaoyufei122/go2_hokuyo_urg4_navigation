import signal
from unittest.mock import Mock

import pytest
from rclpy.signals import SignalHandlerOptions

from go2_base_nav import cmd_vel_bridge, planar_odom


ENTRYPOINTS = (
    (planar_odom, "PlanarOdomNode"),
    (cmd_vel_bridge, "Go2CmdVelBridge"),
)


@pytest.mark.parametrize(("module", "node_class_name"), ENTRYPOINTS)
def test_main_keeps_context_alive_for_shutdown_cleanup(
    monkeypatch,
    module,
    node_class_name,
):
    node = Mock()
    init = Mock()

    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, "init", init)
    monkeypatch.setattr(module.rclpy, "spin_once", Mock())
    monkeypatch.setattr(module.rclpy, "ok", Mock(return_value=False))
    monkeypatch.setattr(module.rclpy, "shutdown", Mock())

    module.main()

    init.assert_called_once_with(
        args=None,
        signal_handler_options=SignalHandlerOptions.NO,
    )


@pytest.mark.parametrize(("module", "node_class_name"), ENTRYPOINTS)
def test_main_ignores_repeated_sigint_while_cleaning_up(
    monkeypatch,
    module,
    node_class_name,
):
    node = Mock()
    node.destroy_node.side_effect = KeyboardInterrupt
    shutdown = Mock()
    set_signal = Mock()

    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "spin_once", Mock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(module.rclpy, "ok", Mock(return_value=True))
    monkeypatch.setattr(module.rclpy, "shutdown", shutdown)
    monkeypatch.setattr(signal, "signal", set_signal)

    module.main()

    set_signal.assert_called_once_with(signal.SIGINT, signal.SIG_IGN)
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


@pytest.mark.parametrize(("module", "node_class_name"), ENTRYPOINTS)
def test_main_polls_ros_with_a_bounded_timeout(
    monkeypatch,
    module,
    node_class_name,
):
    node = Mock()
    spin_once = Mock()

    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "spin", Mock())
    monkeypatch.setattr(module.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(
        module.rclpy,
        "ok",
        Mock(side_effect=(True, False, False)),
    )
    monkeypatch.setattr(module.rclpy, "shutdown", Mock())

    module.main()

    spin_once.assert_called_once_with(node, timeout_sec=0.1)


@pytest.mark.parametrize("context_ok", (False, True))
def test_bridge_only_sends_final_stop_while_context_is_valid(
    monkeypatch,
    context_ok,
):
    bridge = cmd_vel_bridge.Go2CmdVelBridge.__new__(cmd_vel_bridge.Go2CmdVelBridge)
    bridge._sport_client = Mock()
    base_destroy = Mock(return_value=True)

    monkeypatch.setattr(cmd_vel_bridge.rclpy, "ok", Mock(return_value=context_ok))
    monkeypatch.setattr(cmd_vel_bridge.Node, "destroy_node", base_destroy)

    assert bridge.destroy_node() is True

    if context_ok:
        bridge._sport_client.stop_move.assert_called_once_with()
    else:
        bridge._sport_client.stop_move.assert_not_called()
    base_destroy.assert_called_once_with()
