#!/usr/bin/env python3
"""Force voice module to speak — tries I2C@0x30, UART frames, all plausible buses/ports."""

from __future__ import annotations

import argparse
import fcntl
import glob
import struct
import sys
import time
from typing import Iterable

try:
    import serial
except ImportError:
    serial = None

I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707

# Yahboom CI1302 / STC8H — trigger module speaker (passive / reply phrases)
PLAY_FRAMES: list[tuple[str, bytes]] = [
    ('welcome', bytes([0xAA, 0x55, 0x01, 0x00, 0xFB])),
    ('forward_reply', bytes([0xAA, 0x55, 0x00, 0x03, 0xFB])),
    ('stop_reply', bytes([0xAA, 0x55, 0x00, 0x11, 0xFB])),
    ('passive_ff03', bytes([0xAA, 0x55, 0xFF, 0x03, 0xFB])),
    ('passive_ff04', bytes([0xAA, 0x55, 0xFF, 0x04, 0xFB])),
]

I2C_ADDRS = [0x30, 0x31, 0x32, 0x33, 0x2B]
I2C_REGS = [0x00, 0x01, 0x02, 0x03]
I2C_IDS = [0x00, 0x03, 0x04, 0x11]


def i2c_raw_write(bus: int, addr: int, data: bytes) -> None:
    import ctypes

    class i2c_msg(ctypes.Structure):
        _fields_ = [
            ('addr', ctypes.c_uint16),
            ('flags', ctypes.c_uint16),
            ('len', ctypes.c_uint16),
            ('buf', ctypes.c_void_p),
        ]

    class i2c_rdwr_ioctl_data(ctypes.Structure):
        _fields_ = [('msgs', ctypes.POINTER(i2c_msg)), ('nmsgs', ctypes.c_uint32)]

    fd = open(f'/dev/i2c-{bus}', 'wb', buffering=0)
    buf = (ctypes.c_char * len(data))(*data)
    msg = i2c_msg(addr, 0, len(data), ctypes.addressof(buf))
    rdwr = i2c_rdwr_ioctl_data(ctypes.pointer(msg), 1)
    fcntl.ioctl(fd, I2C_RDWR, rdwr)
    fd.close()


def i2c_smbus_byte(bus: int, addr: int, reg: int, val: int) -> None:
    import ctypes

    I2C_SMBUS = 0x0720
    I2C_SMBUS_WRITE = 0
    I2C_SMBUS_BYTE_DATA = 2

    class Data(ctypes.Union):
        _fields_ = [('byte', ctypes.c_uint8)]

    class Args(ctypes.Structure):
        _fields_ = [
            ('read_write', ctypes.c_char),
            ('command', ctypes.c_uint8),
            ('size', ctypes.c_int),
            ('data', ctypes.c_void_p),
        ]

    fd = open(f'/dev/i2c-{bus}', 'wb', buffering=0)
    fcntl.ioctl(fd, I2C_SLAVE, addr)
    d = Data()
    d.byte = val & 0xFF
    a = Args(I2C_SMBUS_WRITE, reg & 0xFF, I2C_SMBUS_BYTE_DATA, ctypes.addressof(d))
    fcntl.ioctl(fd, I2C_SMBUS, a)
    fd.close()


def serial_ports() -> list[str]:
    ports = sorted(glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyACM*'))
    # Prefer likely voice UART first (S1/S5), skip console S0
    order = ['/dev/ttyS1', '/dev/ttyS5', '/dev/ttyACM1', '/dev/ttyACM0']
    ranked = [p for p in order if p in ports]
    ranked += [p for p in ports if p not in ranked and p != '/dev/ttyS0']
    return ranked


def try_serial(port: str, baud: int, pause: float) -> int:
    if serial is None:
        print('pyserial missing', file=sys.stderr)
        return 0
    sent = 0
    try:
        with serial.Serial(port, baud, timeout=0.3, write_timeout=2) as ser:
            ser.reset_input_buffer()
            for name, frame in PLAY_FRAMES:
                n = ser.write(frame)
                ser.flush()
                print(f'  UART {port}@{baud} {name}: wrote {n} {frame.hex()}')
                sent += 1
                time.sleep(pause)
    except Exception as exc:
        print(f'  UART {port}@{baud}: {exc}')
    return sent


def try_i2c(bus: int, pause: float) -> int:
    ok = 0
    for addr in I2C_ADDRS:
        for reg, vid in ((r, i) for r in I2C_REGS for i in I2C_IDS):
            try:
                i2c_smbus_byte(bus, addr, reg, vid)
                print(f'  I2C bus={bus} addr=0x{addr:02x} reg=0x{reg:02x} id=0x{vid:02x} OK')
                ok += 1
                time.sleep(pause)
            except OSError:
                pass
        for name, frame in PLAY_FRAMES:
            try:
                i2c_raw_write(bus, addr, frame)
                print(f'  I2C bus={bus} addr=0x{addr:02x} frame {name} OK')
                ok += 1
                time.sleep(pause)
            except OSError:
                pass
    return ok


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Force Yahboom voice module to play')
    parser.add_argument('--pause', type=float, default=2.5, help='Seconds between attempts')
    parser.add_argument('--serial-only', action='store_true')
    parser.add_argument('--i2c-only', action='store_true')
    args = parser.parse_args(list(argv) if argv is not None else None)

    print('语音模块强制发声测试 — 请听模块喇叭（不是电脑音箱）')
    print('若仍无声：检查 PH2.0 四芯线是否接到 RDK 扩展口 I2C 或 UART\n')

    if not args.i2c_only:
        print('=== UART (115200 + 9600) ===')
        for port in serial_ports():
            for baud in (115200, 9600):
                try_serial(port, baud, args.pause)

    if not args.serial_only:
        print('=== I2C (addr 0x30 等) ===')
        for bus in (5, 2, 4, 6, 0):
            n = try_i2c(bus, args.pause)
            if n:
                print(f'  bus {bus}: {n} writes accepted')

    print('\n完成。若完全无声，请对模块说「小亚小亚」——若有「我在」则模块正常、仅主机线未接对。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
