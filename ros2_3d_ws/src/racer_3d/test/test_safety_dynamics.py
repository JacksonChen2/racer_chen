import numpy as np

from racer_3d.crazyflie import GRAVITY, MASS, velocity_wrench
from racer_3d.safety import (
    aabb_obstacle_filter,
    cbf_swarm_filter,
    esdf_obstacle_filter,
    predicted_path_conflict,
)
from racer_3d.scenario import Box3D


def test_3d_cbf_moves_away_in_vertical_near_miss():
    safe = cbf_swarm_filter(
        preferred=(0.0, 0.0, 0.8),
        position=(0.0, 0.0, 0.8),
        peers=[(1, (0.0, 0.0, 1.25), (0.0, 0.0, -0.2))],
        safe_distance=0.7,
        speed_limit=1.0,
    )
    assert safe[2] < 0.0
    assert np.linalg.norm(safe) <= 1.0 + 1.0e-9


def test_predicted_path_conflict_is_xyz():
    assert predicted_path_conflict(
        [(1.0, 0.0, 0.0, 0.5)],
        [(1.1, 0.0, 0.0, 0.7)],
        0.4,
    )
    assert not predicted_path_conflict(
        [(1.0, 0.0, 0.0, 0.5)],
        [(1.1, 0.0, 0.0, 1.5)],
        0.4,
    )


def test_crazyflie_hover_wrench_and_motor_limits():
    wrench = velocity_wrench(
        desired_velocity=(0.0, 0.0, 0.0),
        current_velocity=(0.0, 0.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
        desired_yaw=0.0,
    )
    assert abs(wrench.local_force[2] - MASS * GRAVITY) < 1.0e-5
    assert np.all(wrench.motor_thrusts >= 0.0)
    assert np.all(wrench.motor_thrusts <= 0.16)
    assert np.max(wrench.motor_thrusts) - np.min(wrench.motor_thrusts) < 1.0e-8


def test_motor_saturation_preserves_collective_thrust():
    from racer_3d.crazyflie import allocate_motors

    motors = allocate_motors(0.32, (0.01, -0.01, 0.004))
    assert np.all(motors >= 0.0)
    assert np.all(motors <= 0.16)
    assert abs(float(np.sum(motors)) - 0.32) < 1.0e-9


def test_rate_feedback_rotates_world_rate_to_body_frame():
    half = np.sqrt(0.5)
    wrench = velocity_wrench(
        desired_velocity=(0.0, 0.0, 0.0),
        current_velocity=(0.0, 0.0, 0.0),
        quaternion_wxyz=(half, 0.0, 0.0, half),
        angular_velocity=(1.0, 0.0, 0.0),
        desired_yaw=np.pi / 2.0,
    )
    # At +90 degree yaw, world +X is body -Y. Damping therefore commands
    # positive body-Y torque, with no roll torque.
    assert abs(wrench.local_torque[0]) < 1.0e-9
    assert wrench.local_torque[1] > 0.0


def test_esdf_filter_accounts_for_rigid_body_stopping_distance():
    class FlatWallMap:
        @staticmethod
        def distance_at(_position, unknown_is_occupied=False):
            assert not unknown_is_occupied
            return 0.50

        @staticmethod
        def esdf_gradient(_position, unknown_is_occupied=False):
            assert not unknown_is_occupied
            return np.asarray((1.0, 0.0, 0.0))

    safe = esdf_obstacle_filter(
        preferred=(-0.7, 0.0, 0.0),
        position=(0.5, 0.0, 0.0),
        voxel_map=FlatWallMap(),
        clearance=0.32,
        speed_limit=0.7,
        current_velocity=(-0.7, 0.0, 0.0),
    )
    assert safe[0] > 0.0


def test_high_rate_aabb_filter_brakes_before_floor_and_wall():
    obstacles = (
        Box3D((0.0, 0.0, -0.05), (10.0, 10.0, 0.1), "floor"),
        Box3D((1.05, 0.0, 1.0), (0.1, 10.0, 2.0), "wall"),
    )
    safe = aabb_obstacle_filter(
        preferred=(0.35, 0.0, -0.35),
        position=(0.60, 0.0, 0.40),
        obstacles=obstacles,
        clearance=0.28,
        speed_limit=0.35,
        current_velocity=(0.30, 0.0, -0.30),
    )
    assert safe[0] < 0.05
    assert safe[2] > -0.05
    assert np.linalg.norm(safe) <= 0.35 + 1.0e-9
