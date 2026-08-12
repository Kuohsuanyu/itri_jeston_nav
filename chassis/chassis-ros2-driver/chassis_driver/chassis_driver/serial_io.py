"""
Handle RS485 serial communication with the VCU.

This includes packet encoding, decoding, and background I/O.
"""

import struct
import threading
import time

import serial

START_BYTE = 0x0A
END_BYTE = 0xFF

TX_LENGTH = 11
RX_LENGTH = 21

# '>' 大端序
# B  起始位 (uint8)
# h  馬達1 目標轉速 (int16)
# h  馬達2 目標轉速 (int16)
# B  Alarm 清除 (uint8)
# 4s 保留 (4 bytes)
# B  結束位 (uint8)
TX_STRUCT = struct.Struct(">BhhB4sB")

# '>' 大端序
# B  起始位 (uint8)
# h  馬達1 實際轉速 (int16)   H 馬達1 霍爾值 (uint16)   B 馬達1 Alarm (uint8)
# h  馬達2 實際轉速 (int16)   H 馬達2 霍爾值 (uint16)   B 馬達2 Alarm (uint8)
# H  電池電壓原始值 (uint16)  h 電池電流原始值 (int16)
# B  SoC (uint8)             B SoH (uint8)
# B  車載資訊 (uint8)[位元遮罩]
# B  保留
# B  通訊狀態計數 (uint8)
# B  結束位 (uint8)
RX_STRUCT = struct.Struct(">BhHBhHBHhBBBBBB")


class ChassisSerial:

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float,
        send_interval: float,
        debug: bool = False,  ##! 檢修485通訊用
    ):
        self._send_interval = send_interval
        self._debug = debug

        self._conn = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

        self._lock = threading.Lock()
        self._target_left_rpm = 0
        self._target_right_rpm = 0
        self._clear_alarm_flag = False
        self._latest_state = None

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        self._conn.close()

    def set_target_rpm(self, left_rpm: int, right_rpm: int, clear_alarm: bool = False):
        with self._lock:
            self._target_left_rpm = left_rpm
            self._target_right_rpm = right_rpm
            self._clear_alarm_flag = clear_alarm

    def get_latest_state(self) -> dict | None:
        """Return the most recent decoded RX state or None if no valid packet has been received."""
        with self._lock:
            return self._latest_state

    def _run(self):
        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            with self._lock:
                left_rpm = self._target_left_rpm
                right_rpm = self._target_right_rpm
                clear_alarm = self._clear_alarm_flag
                self._clear_alarm_flag = False  # 單次觸發，送出後歸零

            tx_packet = self._encode_tx(left_rpm, right_rpm, clear_alarm)
            self._conn.write(tx_packet)

            if self._debug:
                print(
                    f"[TX] {tx_packet.hex(' ').upper()} | "
                    f"L={left_rpm:+d} R={right_rpm:+d} clr={int(clear_alarm)}"
                )

            rx_bytes = self._conn.read(RX_LENGTH)
            state = self._decode_rx(rx_bytes)
            if state is not None:
                with self._lock:
                    self._latest_state = state

            if self._debug:
                if state is not None:
                    print(
                        f"[RX] {rx_bytes.hex(' ').upper()} | "
                        f"L={state['left_rpm']:+d} R={state['right_rpm']:+d} "
                        f"V={state['battery_voltage']:.2f}V "
                        f"I={state['battery_current']:+.2f}A "
                        f"SoC={state['battery_soc']}% seq={state['seq_count']}"
                    )
                else:
                    print(
                        f"[RX] invalid ({len(rx_bytes)} bytes): "
                        f"{rx_bytes.hex(' ').upper()}"
                    )

            elapsed = time.monotonic() - loop_start
            sleep_time = self._send_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    @staticmethod
    def _encode_tx(left_rpm: int, right_rpm: int, clear_alarm: bool) -> bytes:
        return TX_STRUCT.pack(
            START_BYTE,
            left_rpm,
            right_rpm,
            0x01 if clear_alarm else 0x00,
            b"\x00\x00\x00\x00",
            END_BYTE,
        )

    @staticmethod
    def _decode_rx(raw: bytes) -> dict | None:
        if len(raw) != RX_LENGTH:
            return None

        unpacked = RX_STRUCT.unpack(raw)
        (start, m1_rpm, m1_hall, m1_alarm,
         m2_rpm, m2_hall, m2_alarm,
         voltage_raw, current_raw, soc, soh,
         vehicle_status, _reserved_byte18, seq_count, end) = unpacked

        if start != START_BYTE or end != END_BYTE:
            return None

        # 解析車載資訊位元遮罩
        emergency_stop = bool(vehicle_status & 0x01)
        handle_offline = bool(vehicle_status & 0x02)
        driver1_offline = bool(vehicle_status & 0x04)
        driver2_offline = bool(vehicle_status & 0x08)

        return {
            "left_rpm": m1_rpm,
            "left_hall": m1_hall,
            "left_alarm": bool(m1_alarm),
            "right_rpm": m2_rpm,
            "right_hall": m2_hall,
            "right_alarm": bool(m2_alarm),
            "battery_voltage": voltage_raw * 0.01,
            "battery_current": current_raw * 0.01,
            "battery_soc": soc,
            "battery_soh": soh,
            "seq_count": seq_count,
            "emergency_stop": emergency_stop,
            "handle_offline": handle_offline,
            "driver1_offline": driver1_offline,
            "driver2_offline": driver2_offline,
        }
