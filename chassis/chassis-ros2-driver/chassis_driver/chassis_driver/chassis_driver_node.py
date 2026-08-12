"""ROS2 節點主體：整合 serial_io 與 kinematics，對外提供標準 ROS2 介面."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from chassis_driver.serial_io import ChassisSerial
from chassis_driver.kinematics import DifferentialDriveKinematics, OdometryIntegrator
from chassis_msgs.msg import MotorState, ChassisStatus


class ChassisDriverNode(Node):
    def __init__(self):
        super().__init__("chassis_driver")

        self._declare_parameters()
        params = self._read_parameters()

        self._kinematics = DifferentialDriveKinematics(
            wheel_radius=params["wheel_radius"],
            wheel_separation=params["wheel_separation"],
            gear_ratio=params["gear_ratio"],
            cmd_left_direction=params["cmd_left_direction"],
            cmd_right_direction=params["cmd_right_direction"],
            fb_left_direction=params["fb_left_direction"],
            fb_right_direction=params["fb_right_direction"],
        )
        self._odom_integrator = OdometryIntegrator()
        self._publish_tf = params["publish_tf"]
        self._cmd_vel_timeout = params["cmd_vel_timeout"]

        self._serial = ChassisSerial(
            port=params["port"],
            baudrate=params["baudrate"],
            timeout=params["serial_timeout"],
            send_interval=params["send_interval"],
        )
        self._serial.start()

        self._last_cmd_vel_time = self.get_clock().now()
        self._last_state_time = self.get_clock().now()
        self._last_seq_count = None
        self._wheel_angle_left = 0.0
        self._wheel_angle_right = 0.0
        self._lost_packet_count = 0

        self._tf_broadcaster = TransformBroadcaster(self)

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._motor_state_pub = self.create_publisher(MotorState, "chassis/motor_state", 10)
        self._chassis_status_pub = self.create_publisher(ChassisStatus, "chassis/status", 10)

        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self.create_service(Trigger, "clear_alarm", self._clear_alarm_callback)

        self._clear_alarm_pending = False

        self.create_timer(params["send_interval"], self._control_loop)

    def _declare_parameters(self):
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("serial_timeout", 0.5)
        self.declare_parameter("send_interval", 0.1)
        self.declare_parameter("gear_ratio", 30.0)
        self.declare_parameter("wheel_radius", 0.2032)
        self.declare_parameter("wheel_separation", 0.6221)
        self.declare_parameter("cmd_left_direction", 1)
        self.declare_parameter("cmd_right_direction", 1)
        self.declare_parameter("fb_left_direction", -1)
        self.declare_parameter("fb_right_direction", -1)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("cmd_vel_timeout", 0.5)

    def _read_parameters(self) -> dict:
        return {
            "port": self.get_parameter("port").value,
            "baudrate": self.get_parameter("baudrate").value,
            "serial_timeout": self.get_parameter("serial_timeout").value,
            "send_interval": self.get_parameter("send_interval").value,
            "gear_ratio": self.get_parameter("gear_ratio").value,
            "wheel_radius": self.get_parameter("wheel_radius").value,
            "wheel_separation": self.get_parameter("wheel_separation").value,
            "cmd_left_direction": self.get_parameter("cmd_left_direction").value,
            "cmd_right_direction": self.get_parameter("cmd_right_direction").value,
            "fb_left_direction": self.get_parameter("fb_left_direction").value,
            "fb_right_direction": self.get_parameter("fb_right_direction").value,
            "publish_tf": self.get_parameter("publish_tf").value,
            "cmd_vel_timeout": self.get_parameter("cmd_vel_timeout").value,
        }

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()
        left_rpm, right_rpm = self._kinematics.cmd_vel_to_motor_rpm(
            msg.linear.x, msg.angular.z
        )
        self._serial.set_target_rpm(left_rpm, right_rpm)

    def _clear_alarm_callback(self, request, response):
        self._clear_alarm_pending = True
        response.success = True
        response.message = "Alarm clear requested"
        return response

    def _check_packet_loss(self, state: dict) -> int:
        """Compare sequence counts and return the number of lost packets."""
        current_seq = state["seq_count"]

        if self._last_seq_count is None:
            self._last_seq_count = current_seq
            return 0

        expected_seq = (self._last_seq_count + 1) % 256
        lost_this_cycle = (current_seq - expected_seq) % 256

        self._last_seq_count = current_seq
        self._lost_packet_count += lost_this_cycle

        return lost_this_cycle

    def _control_loop(self):
        now = self.get_clock().now()

        # cmd_vel watchdog：逾時則強制歸零，避免斷線暴衝
        elapsed_since_cmd = (now - self._last_cmd_vel_time).nanoseconds / 1e9
        if elapsed_since_cmd > self._cmd_vel_timeout:
            self._serial.set_target_rpm(0, 0, clear_alarm=self._clear_alarm_pending)
            self._clear_alarm_pending = False
        elif self._clear_alarm_pending:
            # 有 pending 的清除請求但尚未逾時，仍需把 clear_alarm 帶上下一筆
            self._serial.set_target_rpm(0, 0, clear_alarm=True)
            self._clear_alarm_pending = False

        state = self._serial.get_latest_state()
        if state is None:
            self._publish_diagnostics(connected=False)
            return

        dt = (now - self._last_state_time).nanoseconds / 1e9
        self._last_state_time = now

        lost_packets = self._check_packet_loss(state)

        self._publish_odom(state, dt, now)
        self._publish_joint_states(state, dt, now)
        self._publish_battery_state(state, now)
        self._publish_motor_state(state)
        self._publish_chassis_status(state)
        self._publish_diagnostics(connected=True, state=state, lost_packets=lost_packets)

    def _publish_odom(self, state: dict, dt: float, stamp):
        v_left, v_right = self._kinematics.motor_rpm_to_wheel_speed(
            state["left_rpm"], state["right_rpm"]
        )
        vx, wz = self._kinematics.wheel_speed_to_vx_wz(v_left, v_right)
        self._odom_integrator.integrate(vx, wz, dt)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._odom_integrator.x
        odom.pose.pose.position.y = self._odom_integrator.y
        odom.pose.pose.orientation.z = math.sin(self._odom_integrator.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom_integrator.theta / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp.to_msg()
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_footprint"
            tf.transform.translation.x = self._odom_integrator.x
            tf.transform.translation.y = self._odom_integrator.y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self._tf_broadcaster.sendTransform(tf)

    def _publish_joint_states(self, state: dict, dt: float, stamp):
        """Publish wheel joint angles and angular velocities."""
        left_vel, right_vel = self._kinematics.motor_rpm_to_wheel_angular_velocity(
            state["left_rpm"], state["right_rpm"]
        )
        self._wheel_angle_left += left_vel * dt
        self._wheel_angle_right += right_vel * dt

        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = [
            "wheel_front_left_joint",
            "wheel_rear_left_joint",
            "wheel_front_right_joint",
            "wheel_rear_right_joint",
        ]
        msg.position = [
            self._wheel_angle_left,
            self._wheel_angle_left,
            self._wheel_angle_right,
            self._wheel_angle_right,
        ]
        msg.velocity = [left_vel, left_vel, right_vel, right_vel]
        self._joint_state_pub.publish(msg)

    def _publish_battery_state(self, state: dict, stamp):
        msg = BatteryState()
        msg.header.stamp = stamp.to_msg()
        msg.voltage = state["battery_voltage"]
        msg.current = state["battery_current"]
        msg.percentage = float(state["battery_soc"])
        msg.present = True
        self._battery_pub.publish(msg)

    def _publish_motor_state(self, state: dict):
        msg = MotorState()
        msg.left_rpm = state["left_rpm"]
        msg.left_hall = state["left_hall"]
        msg.left_alarm = state["left_alarm"]
        msg.right_rpm = state["right_rpm"]
        msg.right_hall = state["right_hall"]
        msg.right_alarm = state["right_alarm"]
        self._motor_state_pub.publish(msg)

    def _publish_chassis_status(self, state: dict):
        """發布車載安全狀態（緊急開關、手把與驅動器連線狀態）。"""
        msg = ChassisStatus()
        msg.emergency_stop = state["emergency_stop"]
        msg.handle_offline = state["handle_offline"]
        msg.driver1_offline = state["driver1_offline"]
        msg.driver2_offline = state["driver2_offline"]
        self._chassis_status_pub.publish(msg)

    def _publish_diagnostics(
            self, connected: bool, state: dict | None = None, lost_packets: int = 0
        ):
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()

            status = DiagnosticStatus()
            status.name = "chassis_driver: VCU link"

            if not connected:
                status.level = DiagnosticStatus.ERROR
                status.message = "No valid packet received from VCU"
            elif state["emergency_stop"]:
                status.level = DiagnosticStatus.ERROR
                status.message = "Emergency stop is active"
            elif state["driver1_offline"] or state["driver2_offline"]:
                status.level = DiagnosticStatus.ERROR
                status.message = "Motor driver offline"
                status.values.append(
                    KeyValue(key="driver1_offline", value=str(state["driver1_offline"]))
                )
                status.values.append(
                    KeyValue(key="driver2_offline", value=str(state["driver2_offline"]))
                )
            elif state["handle_offline"]:
                status.level = DiagnosticStatus.WARN
                status.message = "Remote handle offline"
            elif state["left_alarm"] or state["right_alarm"]:
                status.level = DiagnosticStatus.WARN
                status.message = "Motor alarm active"
                status.values.append(
                    KeyValue(key="left_alarm", value=str(state["left_alarm"]))
                )
                status.values.append(
                    KeyValue(key="right_alarm", value=str(state["right_alarm"]))
                )
            elif lost_packets > 0:
                status.level = DiagnosticStatus.WARN
                status.message = f"Packet loss detected: {lost_packets} packet(s) this cycle"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Normal"

            status.values.append(
                KeyValue(key="total_lost_packets", value=str(self._lost_packet_count))
            )

            array.status.append(status)
            self._diagnostics_pub.publish(array)

    def destroy_node(self):
        self._serial.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
