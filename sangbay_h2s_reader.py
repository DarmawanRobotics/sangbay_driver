"""
Sangbay S-M4T Gas Sensor - H2S Reader via USB-to-RS485
=======================================================
Sensor default address : 0xFF
Baud rate              : 9600, 8N1
Gas target             : H2S (code 0x02)
"""

import serial
import struct
import time
from typing import Optional


# ── CRC-16 (Modbus) ──────────────────────────────────────────────────────────

CRC_TABLE = [
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
]

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = CRC_TABLE[(byte ^ crc) & 0x0F] ^ (crc >> 4)
        crc = CRC_TABLE[((byte >> 4) ^ crc) & 0x0F] ^ (crc >> 4)
    return crc


def append_crc(frame: list) -> bytes:
    raw = bytes(frame)
    c = crc16(raw)
    return raw + bytes([c & 0xFF, (c >> 8) & 0xFF])


def hex_dump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


# ── Low-level Modbus helpers ──────────────────────────────────────────────────

def read_register(
    ser: serial.Serial,
    sensor_id: int,
    reg_addr: int,
    reg_count: int = 1
) -> Optional[bytes]:

    frame = append_crc([
        sensor_id,
        0x03,
        (reg_addr >> 8) & 0xFF,
        reg_addr & 0xFF,
        (reg_count >> 8) & 0xFF,
        reg_count & 0xFF,
    ])

    print(f"\n[TX] {hex_dump(frame)}")

    ser.reset_input_buffer()
    ser.write(frame)

    expected = 1 + 1 + 1 + 2 * reg_count + 2
    resp = ser.read(expected)

    if len(resp) == 0:
        print("[RX] <NO RESPONSE>")
        return None

    print(f"[RX] {hex_dump(resp)}")

    if len(resp) < expected:
        print(f"[WARN] Incomplete frame ({len(resp)}/{expected})")
        return None

    recv_crc = struct.unpack_from('<H', resp, len(resp) - 2)[0]
    calc_crc = crc16(resp[:-2])
    if recv_crc != calc_crc:
        print(f"[CRC ERROR] recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")
        return None

    if resp[1] & 0x80:
        print(f"[MODBUS ERROR] code=0x{resp[2]:02X}")
        return None

    byte_count = resp[2]
    payload = resp[3: 3 + byte_count]

    print(f"[DATA] {hex_dump(payload)}")

    return payload


# ── High-level sensor helpers ─────────────────────────────────────────────────

def read_decimal_places(ser: serial.Serial, sensor_id: int) -> int:
    data = read_register(ser, sensor_id, 0x2031)
    if data is None:
        return 0
    return struct.unpack('>H', data)[0]


def read_concentration_raw(
    ser: serial.Serial,
    sensor_id: int
) -> Optional[int]:
    data = read_register(ser, sensor_id, 0x6001)
    if data is None:
        return None
    return struct.unpack('>H', data)[0]


def read_alarm_status(
    ser: serial.Serial,
    sensor_id: int
) -> Optional[int]:
    data = read_register(ser, sensor_id, 0x6000)
    if data is None:
        return None
    return struct.unpack('>H', data)[0]


def read_gas_name_code(
    ser: serial.Serial,
    sensor_id: int
) -> Optional[int]:
    data = read_register(ser, sensor_id, 0x2027)
    if data is None:
        return None
    return struct.unpack('>H', data)[0]


# ── Alarm decoder ─────────────────────────────────────────────────────────────

def decode_alarm(word: int) -> str:
    if not (word & 0x8000):
        countdown = (word & 0x7FFF) * 0.1
        return f"PREHEATING ({countdown:.1f}s)"

    flags = []
    if word & (1 << 6): flags.append("HIGH-PROTECTION")
    if word & (1 << 5): flags.append("SENSOR-ERROR")
    if word & (1 << 4): flags.append("OVER-RANGE")
    if word & (1 << 3): flags.append("TWA")
    if word & (1 << 2): flags.append("STEL")
    if word & (1 << 1): flags.append("HIGH")
    if word & (1 << 0): flags.append("LOW")

    return ", ".join(flags) if flags else "NORMAL"


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    PORT = "/dev/ttyUSB0"   # ganti sesuai device kamu
    BAUD = 9600
    SENSOR_ID = 0xFF
    INTERVAL = 2.0

    print("=" * 60)
    print("Sangbay S-M4T H2S Monitor")
    print("=" * 60)

    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open port: {e}")
        return

    with ser:
        gas_code = read_gas_name_code(ser, SENSOR_ID)
        if gas_code == 0x02:
            print("Gas: H2S ✅")
        else:
            print(f"[WARNING] Gas code: {gas_code}")

        decimals = read_decimal_places(ser, SENSOR_ID)
        divisor = 10 ** decimals

        print(f"Decimals: {decimals}")
        print("-" * 60)
        print("Time       H2S(ppm)     Status")
        print("-" * 60)

        try:
            while True:
                raw = read_concentration_raw(ser, SENSOR_ID)
                alarm = read_alarm_status(ser, SENSOR_ID)

                ts = time.strftime("%H:%M:%S")

                if raw is None:
                    print(f"{ts}    --          NO DATA")
                else:
                    conc = raw / divisor
                    status = decode_alarm(alarm) if alarm else "?"
                    print(f"{ts}    {conc:8.2f}    {status}")

                time.sleep(INTERVAL)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()