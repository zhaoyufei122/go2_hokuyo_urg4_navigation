import json
from typing import Any, Mapping

from rclpy.node import Node
from unitree_api.msg import Request


class Go2SportClient:
    MOVE_API_ID = 1008
    STOP_MOVE_API_ID = 1003

    def __init__(
        self,
        node: Node,
        request_topic: str = "/api/sport/request",
        qos_depth: int = 10,
    ) -> None:
        self._publisher = node.create_publisher(Request, request_topic, qos_depth)

    def make_request(
        self,
        api_id: int,
        parameter: Mapping[str, Any] | None = None,
    ) -> Request:
        request = Request()
        request.header.identity.api_id = int(api_id)
        if parameter is not None:
            request.parameter = json.dumps(parameter, separators=(",", ":"))
        return request

    def move(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        request = self.make_request(
            self.MOVE_API_ID,
            {
                "x": float(linear_x),
                "y": float(linear_y),
                "z": float(angular_z),
            },
        )
        self._publisher.publish(request)

    def stop_move(self) -> None:
        self._publisher.publish(self.make_request(self.STOP_MOVE_API_ID))
