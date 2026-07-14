from go2_base_nav.sport_client import Go2SportClient


def test_move_request_uses_compact_json():
    client = Go2SportClient.__new__(Go2SportClient)
    request = client.make_request(1008, {"x": 0.4, "y": 0.0, "z": -0.4})
    assert request.header.identity.api_id == 1008
    assert request.parameter == '{"x":0.4,"y":0.0,"z":-0.4}'


def test_stop_move_request_has_empty_parameter():
    client = Go2SportClient.__new__(Go2SportClient)
    request = client.make_request(1003)
    assert request.header.identity.api_id == 1003
    assert request.parameter == ""
