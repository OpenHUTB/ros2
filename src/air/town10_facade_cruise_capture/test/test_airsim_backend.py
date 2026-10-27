"""仅验证RPC协议和保护分支，不代表原生场景飞行实验。"""

import io
from types import SimpleNamespace as NS

import numpy as np
import pytest
from PIL import Image

from town10_capture.airsim_backend import AirSimBackend, quaternion_matrix
from town10_capture.config import CruiseConfig


class Future:
    def join(self):
        pass


class MockClient:
    def __init__(self):
        self.commands = []
        self.collided = False
        self.released = False
        self.hovered = 0
        self.landed = 1
        self.depth = 4.0
        self.collision_stamp = 0
        self.rgb = np.array(
            [[[255, 0, 0], [0, 0, 255]], [[17, 34, 51], [200, 100, 25]]], dtype=np.uint8
        )
        self.image_requests = []

    def confirmConnection(self):
        pass

    def enableApiControl(self, value, vehicle_name=""):
        self.released = not value

    def hoverAsync(self, **kwargs):
        self.hovered += 1
        return Future()

    def getMultirotorState(self, **kwargs):
        return NS(
            timestamp=123,
            landed_state=self.landed,
            kinematics_estimated=NS(
                position=NS(x_val=1, y_val=2, z_val=-3),
                linear_velocity=NS(x_val=0.1, y_val=0.2, z_val=0),
                orientation=NS(x_val=0, y_val=0, z_val=0, w_val=1),
            ),
        )

    def moveByVelocityAsync(self, *args, **kwargs):
        self.commands.append((args, kwargs))

    def simGetCollisionInfo(self, **kwargs):
        return NS(has_collided=self.collided, time_stamp=self.collision_stamp)

    def simGetImages(self, *args, **kwargs):
        self.image_requests = args[0]
        stream = io.BytesIO()
        Image.fromarray(self.rgb).save(stream, format="PNG")
        return [
            NS(width=2, height=2, image_data_uint8=stream.getvalue(), time_stamp=456),
            NS(width=3, height=3, image_data_float=[self.depth] * 9, time_stamp=457),
        ]


API = NS(
    LandedState=NS(Landed=0),
    DrivetrainType=NS(MaxDegreeOfFreedom=0),
    YawMode=lambda *a: a,
    ImageRequest=lambda *a: a,
    ImageType=NS(Scene=0, DepthPlanar=1),
)


def test_rpc_axes_image_and_exit_hover():
    client = MockClient()
    backend = AirSimBackend(CruiseConfig(backend="airsim"), client, API)
    position, velocity, q = backend.state()
    np.testing.assert_allclose(position, [2, 1, 3])
    np.testing.assert_allclose(velocity, [0.2, 0.1, 0])
    np.testing.assert_allclose(quaternion_matrix(q) @ [0, 0, 1], [0, 0, 1], atol=1e-12)
    backend.step([0.2, 0.1, 0.3])
    np.testing.assert_allclose(client.commands[-1][0], [0.1, 0.2, -0.3])
    assert backend.image().shape == (2, 2, 3)
    np.testing.assert_array_equal(backend.image(), client.rgb)
    assert client.image_requests[0] == ("0", API.ImageType.Scene, False, True)
    assert client.image_requests[1] == ("0", API.ImageType.DepthPlanar, True, False)
    assert backend.last_image_metadata["sensor_timestamp_ns"] == 456
    np.testing.assert_allclose(backend.depth_samples, [4.0] * 9)
    assert backend.depth_statistics["front_depth_min_m"] == 4.0
    backend.close()
    assert client.released and client.hovered == 2


def test_collision_and_nan_command():
    client = MockClient()
    backend = AirSimBackend(CruiseConfig(backend="airsim"), client, API)
    try:
        client.collided = True
        with pytest.raises(RuntimeError, match="碰撞"):
            backend.check_collision()
        with pytest.raises(ValueError):
            backend.step([float("nan"), 0, 0])
        assert not client.commands
    finally:
        backend.close()


def test_requires_explicit_takeoff():
    client = MockClient()
    client.landed = 0
    with pytest.raises(RuntimeError, match="尚未起飞"):
        AirSimBackend(CruiseConfig(backend="airsim"), client, API)
    assert client.released


def test_invalid_depth_does_not_become_valid_measurement():
    client = MockClient()
    client.depth = float("nan")
    with pytest.raises(RuntimeError, match="有效值不足"):
        AirSimBackend(CruiseConfig(backend="airsim"), client, API)
    assert not client.commands
    assert client.released


def test_near_obstacle_stops_before_velocity_rpc(tmp_path):
    client = MockClient()
    client.depth = 1.0
    config = CruiseConfig(backend="airsim", output=tmp_path)
    with pytest.raises(RuntimeError, match="初始前向深度"):
        AirSimBackend(config, client, API)
    assert not client.commands
    assert client.released


def test_historical_collision_is_ignored_but_new_timestamp_stops():
    client = MockClient()
    client.collided = True
    client.collision_stamp = 100
    backend = AirSimBackend(CruiseConfig(backend="airsim"), client, API)
    try:
        backend.check_collision()
        client.collision_stamp = 101
        with pytest.raises(RuntimeError, match="碰撞"):
            backend.check_collision()
    finally:
        backend.close()


def test_hybrid_cancels_adverse_nn_bias_near_waypoint():
    from town10_capture.mission import native_hybrid_velocity

    # 控制律回归：各轴持续逆向NN偏置不应形成约0.3m定常误差。
    position = np.zeros(3)
    target = np.array([2.0, -0.8, 1.5])
    velocity = np.zeros(3)
    for _ in range(1600):
        error = target - position
        adversarial = -np.sign(error)
        command = native_hybrid_velocity(adversarial, error, velocity)
        assert np.max(np.abs(command)) <= 1.0
        velocity += np.clip(4 * (command - velocity), -8, 8) * 0.05
        position += velocity * 0.05
    assert np.linalg.norm(target - position) < 0.07
    np.testing.assert_allclose(
        native_hybrid_velocity([1, -1, 1], [0, 0, 0], [0, 0, 0]), [0, 0, 0]
    )


def test_native_depth_metrics_do_not_claim_four_meter_tracking(tmp_path):
    import json

    from town10_capture.mission import CruiseMission

    client = MockClient()
    client.depth = 7.2
    config = CruiseConfig(backend="airsim", output=tmp_path)
    mission = CruiseMission(config, backend=AirSimBackend(config, client, API))
    mission.tick()
    mission.finish("interrupted")
    metrics = json.loads((tmp_path / "metrics.json").read_text())
    assert "reference_distance_rmse_m" not in metrics
    assert metrics["front_depth_median_mean_m"] == pytest.approx(7.2)
    assert metrics["front_depth_min_m"] == pytest.approx(7.2)
    assert metrics["route_error_rmse_m"] == pytest.approx(0)
