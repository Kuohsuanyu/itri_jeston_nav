"""差速驅動運動學: cmd_vel <-> 雙輪馬達 RPM 互換."""

import math


class DifferentialDriveKinematics:
    """
    差速驅動運動學計算.

    座標慣例：vx > 0 為前進，wz > 0 為逆時鐘（左轉）。
    參數：wheel_radius、wheel_separation、gear_ratio、
    cmd_left_direction、cmd_right_direction、fb_left_direction、
    fb_right_direction。
    """

    def __init__(
        self,
        wheel_radius: float,
        wheel_separation: float,
        gear_ratio: float,
        cmd_left_direction: int = 1,
        cmd_right_direction: int = 1,
        fb_left_direction: int = 1,
        fb_right_direction: int = 1,
    ):
        self._r = wheel_radius
        self._l = wheel_separation
        self._n = gear_ratio
        self._cmd_left_dir = cmd_left_direction
        self._cmd_right_dir = cmd_right_direction
        self._fb_left_dir = fb_left_direction
        self._fb_right_dir = fb_right_direction

    def cmd_vel_to_motor_rpm(self, vx: float, wz: float) -> tuple[int, int]:
        """Convert cmd_vel (vx, wz) to left/right motor RPM."""
        v_left = vx - wz * self._l / 2.0
        v_right = vx + wz * self._l / 2.0

        wheel_rpm_left = (v_left / self._r) * 60.0 / (2.0 * math.pi)
        wheel_rpm_right = (v_right / self._r) * 60.0 / (2.0 * math.pi)

        motor_rpm_left = round(wheel_rpm_left * self._n * self._cmd_left_dir)
        motor_rpm_right = round(wheel_rpm_right * self._n * self._cmd_right_dir)

        return motor_rpm_left, motor_rpm_right

    def motor_rpm_to_wheel_speed(self, left_rpm: int, right_rpm: int) -> tuple[float, float]:
        """Convert actual motor RPM to left/right wheel linear speeds in m/s."""
        wheel_rpm_left = (left_rpm * self._fb_left_dir) / self._n
        wheel_rpm_right = (right_rpm * self._fb_right_dir) / self._n

        v_left = wheel_rpm_left / 60.0 * 2.0 * math.pi * self._r
        v_right = wheel_rpm_right / 60.0 * 2.0 * math.pi * self._r

        return v_left, v_right

    def motor_rpm_to_wheel_angular_velocity(
        self, left_rpm: int, right_rpm: int
    ) -> tuple[float, float]:
        """
        Convert actual motor RPM to left/right wheel angular velocities.

        The values are expressed in rad/s.
        """
        wheel_rpm_left = (left_rpm * self._fb_left_dir) / self._n
        wheel_rpm_right = (right_rpm * self._fb_right_dir) / self._n
        return (
            wheel_rpm_left * 2.0 * math.pi / 60.0,
            wheel_rpm_right * 2.0 * math.pi / 60.0,
        )

    def wheel_speed_to_vx_wz(self, v_left: float, v_right: float) -> tuple[float, float]:
        """Convert left/right wheel linear speeds to chassis vx and wz."""
        vx = (v_left + v_right) / 2.0
        wz = (v_right - v_left) / self._l
        return vx, wz


class OdometryIntegrator:
    """Integrate vx and wz over time to compute x, y, and theta."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def integrate(self, vx: float, wz: float, dt: float):
        self.theta += wz * dt
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
