"""測試 serial_io 模組的封包編碼與解碼邏輯，不需接硬體。"""

import pytest

from chassis_driver.serial_io import ChassisSerial, START_BYTE, END_BYTE, RX_LENGTH


def build_rx_packet(
    m1_rpm=0, m1_hall=0, m1_alarm=0,
    m2_rpm=0, m2_hall=0, m2_alarm=0,
    voltage_raw=0, current_raw=0, soc=0, soh=0,
    vehicle_status=0, reserved_byte18=0,
    seq_count=0,
    start=START_BYTE, end=END_BYTE,
):
    """依 RX_STRUCT 欄位順序組出一筆原始 21 bytes 封包，供測試組裝用。"""
    from chassis_driver.serial_io import RX_STRUCT

    return RX_STRUCT.pack(
        start,
        m1_rpm, m1_hall, m1_alarm,
        m2_rpm, m2_hall, m2_alarm,
        voltage_raw, current_raw, soc, soh,
        vehicle_status, reserved_byte18,
        seq_count,
        end,
    )


# ── _decode_rx：基本欄位解析 ──────────────────────────────

def test_decode_rx_valid_packet_parses_motor_and_battery_fields():
    raw = build_rx_packet(
        m1_rpm=300, m1_hall=1200, m1_alarm=0,
        m2_rpm=-300, m2_hall=1180, m2_alarm=1,
        voltage_raw=2400, current_raw=-50, soc=87, soh=95,
        seq_count=42,
    )
    state = ChassisSerial._decode_rx(raw)

    assert state is not None
    assert state["left_rpm"] == 300
    assert state["left_hall"] == 1200
    assert state["left_alarm"] is False
    assert state["right_rpm"] == -300
    assert state["right_hall"] == 1180
    assert state["right_alarm"] is True
    assert state["battery_voltage"] == pytest.approx(24.0)
    assert state["battery_current"] == pytest.approx(-0.5)
    assert state["battery_soc"] == 87
    assert state["battery_soh"] == 95
    assert state["seq_count"] == 42


def test_decode_rx_wrong_length_returns_none():
    raw = build_rx_packet()[:-1]  # 少一個 byte
    assert ChassisSerial._decode_rx(raw) is None


def test_decode_rx_wrong_start_byte_returns_none():
    raw = build_rx_packet(start=0xFF)
    assert ChassisSerial._decode_rx(raw) is None


def test_decode_rx_wrong_end_byte_returns_none():
    raw = build_rx_packet(end=0x00)
    assert ChassisSerial._decode_rx(raw) is None


# ── _decode_rx：Byte17 車載狀態位元解析 ────────────────────

@pytest.mark.parametrize(
    "vehicle_status, expected",
    [
        (0b0000, {"emergency_stop": False, "handle_offline": False,
                  "driver1_offline": False, "driver2_offline": False}),
        (0b0001, {"emergency_stop": True, "handle_offline": False,
                  "driver1_offline": False, "driver2_offline": False}),
        (0b0010, {"emergency_stop": False, "handle_offline": True,
                  "driver1_offline": False, "driver2_offline": False}),
        (0b0100, {"emergency_stop": False, "handle_offline": False,
                  "driver1_offline": True, "driver2_offline": False}),
        (0b1000, {"emergency_stop": False, "handle_offline": False,
                  "driver1_offline": False, "driver2_offline": True}),
        (0b1111, {"emergency_stop": True, "handle_offline": True,
                  "driver1_offline": True, "driver2_offline": True}),
    ],
)
def test_decode_rx_vehicle_status_bits(vehicle_status, expected):
    raw = build_rx_packet(vehicle_status=vehicle_status)
    state = ChassisSerial._decode_rx(raw)

    assert state["emergency_stop"] == expected["emergency_stop"]
    assert state["handle_offline"] == expected["handle_offline"]
    assert state["driver1_offline"] == expected["driver1_offline"]
    assert state["driver2_offline"] == expected["driver2_offline"]


def test_decode_rx_vehicle_status_ignores_upper_bits():
    """bit4~bit7 目前未定義，不應影響任何已解析的狀態位元。"""
    raw = build_rx_packet(vehicle_status=0b11110000)
    state = ChassisSerial._decode_rx(raw)

    assert state["emergency_stop"] is False
    assert state["handle_offline"] is False
    assert state["driver1_offline"] is False
    assert state["driver2_offline"] is False


# ── _encode_tx ────────────────────────────────────────────

def test_encode_tx_packet_structure():
    packet = ChassisSerial._encode_tx(left_rpm=300, right_rpm=-100, clear_alarm=False)

    assert len(packet) == 11
    assert packet[0] == START_BYTE
    assert packet[-1] == END_BYTE


def test_encode_tx_clear_alarm_flag():
    with_clear = ChassisSerial._encode_tx(left_rpm=0, right_rpm=0, clear_alarm=True)
    without_clear = ChassisSerial._encode_tx(left_rpm=0, right_rpm=0, clear_alarm=False)

    assert with_clear[5] == 0x01
    assert without_clear[5] == 0x00