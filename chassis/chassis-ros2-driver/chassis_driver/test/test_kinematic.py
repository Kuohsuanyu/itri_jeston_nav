"""
純數學層測試，不需要接硬體.

驗證 cmd_vel <-> motor_rpm 的換算，在無方向修正時互為逆運算，並驗證方向修正與邊界案例的行為。
"""

import math

import pytest

from chassis_driver.kinematics import DifferentialDriveKinematics, OdometryIntegrator

WHEEL_RADIUS = 0.2032
WHEEL_SEPARATION = 0.6221
GEAR_RATIO = 30.0


@pytest.fixture
def kinematics():
    return DifferentialDriveKinematics(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
        gear_ratio=GEAR_RATIO,
    )


def test_zero_velocity_gives_zero_rpm(kinematics):
    left_rpm, right_rpm = kinematics.cmd_vel_to_motor_rpm(0.0, 0.0)
    assert left_rpm == 0
    assert right_rpm == 0


def test_pure_forward_gives_equal_rpm(kinematics):
    left_rpm, right_rpm = kinematics.cmd_vel_to_motor_rpm(0.5, 0.0)
    assert left_rpm == right_rpm
    assert left_rpm > 0


def test_pure_rotation_gives_opposite_rpm(kinematics):
    left_rpm, right_rpm = kinematics.cmd_vel_to_motor_rpm(0.0, 1.0)
    assert left_rpm == -right_rpm
    assert left_rpm < 0
    assert right_rpm > 0


@pytest.mark.parametrize(
    "vx, wz",
    [
        (0.5, 0.0),
        (0.3, 0.5),
        (-0.4, 0.0),
        (0.0, -1.2),
        (0.8, -0.3),
    ],
)
def test_round_trip_vx_wz(kinematics, vx, wz):
    """cmd_vel -> motor_rpm -> wheel_speed -> vx/wz should approximately equal the input."""
    left_rpm, right_rpm = kinematics.cmd_vel_to_motor_rpm(vx, wz)
    v_left, v_right = kinematics.motor_rpm_to_wheel_speed(left_rpm, right_rpm)
    vx_out, wz_out = kinematics.wheel_speed_to_vx_wz(v_left, v_right)

    assert vx_out == pytest.approx(vx, abs=1e-3)
    assert wz_out == pytest.approx(wz, abs=1e-3)


def test_direction_correction_flips_sign():
    normal = DifferentialDriveKinematics(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
        gear_ratio=GEAR_RATIO,
        cmd_left_direction=1,
        cmd_right_direction=1,
    )
    flipped = DifferentialDriveKinematics(
        wheel_radius=WHEEL_RADIUS,
        wheel_separation=WHEEL_SEPARATION,
        gear_ratio=GEAR_RATIO,
        cmd_left_direction=-1,
        cmd_right_direction=1,
    )

    left_normal, right_normal = normal.cmd_vel_to_motor_rpm(0.5, 0.0)
    left_flipped, right_flipped = flipped.cmd_vel_to_motor_rpm(0.5, 0.0)

    assert left_flipped == -left_normal
    assert right_flipped == right_normal


def test_motor_rpm_to_wheel_angular_velocity_matches_linear_speed(kinematics):
    left_rpm, right_rpm = 300, 300
    v_left, v_right = kinematics.motor_rpm_to_wheel_speed(left_rpm, right_rpm)
    w_left, w_right = kinematics.motor_rpm_to_wheel_angular_velocity(left_rpm, right_rpm)

    assert v_left == pytest.approx(w_left * WHEEL_RADIUS, abs=1e-6)
    assert v_right == pytest.approx(w_right * WHEEL_RADIUS, abs=1e-6)


def test_odometry_integrator_straight_line():
    integrator = OdometryIntegrator()
    dt = 0.1
    for _ in range(10):
        integrator.integrate(vx=1.0, wz=0.0, dt=dt)

    assert integrator.x == pytest.approx(1.0, abs=1e-9)
    assert integrator.y == pytest.approx(0.0, abs=1e-9)
    assert integrator.theta == pytest.approx(0.0, abs=1e-9)


def test_odometry_integrator_pure_rotation():
    integrator = OdometryIntegrator()
    dt = 0.1
    for _ in range(10):
        integrator.integrate(vx=0.0, wz=math.pi, dt=dt)

    assert integrator.x == pytest.approx(0.0, abs=1e-9)
    assert integrator.y == pytest.approx(0.0, abs=1e-9)
    assert integrator.theta == pytest.approx(math.pi, abs=1e-9)


def test_odometry_integrator_reset():
    integrator = OdometryIntegrator()
    integrator.integrate(vx=1.0, wz=0.5, dt=0.1)
    integrator.reset()

    assert integrator.x == 0.0
    assert integrator.y == 0.0
    assert integrator.theta == 0.0
