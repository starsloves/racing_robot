"""Play preset phrases on Yahboom voice module via I2C (CI13XX / STC8H @ 0x2b)."""

from __future__ import annotations

import ctypes
import fcntl
from typing import Any

try:
    import smbus2
except ImportError:  # pragma: no cover
    smbus2 = None

# Origincar 扩展板: i2cdetect -y -r 5 → 0x2b (STC8H 转发至 CI1302)
# CI13 IIC 被动播报: [0x03, cmd_id, 0x03+cmd_id, 0x5A]
CI13_PLAY_REG = 0x03
CI13_FRAME_TAIL = 0x5A
I2C_RDWR = 0x0707


def build_ci13_play_packet(command_id: int) -> list[int]:
    reg = CI13_PLAY_REG
    cid = int(command_id) & 0xFF
    checksum = (reg + cid) & 0xFF
    return [reg, cid, checksum, CI13_FRAME_TAIL]


def i2c_raw_write(bus: int, addr: int, data: bytes) -> None:
    """Linux i2c_rdwr_ioctl_data — no extra register prefix (matches voice_node.cpp)."""
    class I2cMsg(ctypes.Structure):
        _fields_ = [
            ('addr', ctypes.c_uint16),
            ('flags', ctypes.c_uint16),
            ('len', ctypes.c_uint16),
            ('buf', ctypes.c_void_p),
        ]

    class I2cRdwrIoctlData(ctypes.Structure):
        _fields_ = [('msgs', ctypes.POINTER(I2cMsg)), ('nmsgs', ctypes.c_uint32)]

    fd = open(f'/dev/i2c-{bus}', 'wb', buffering=0)
    try:
        buf = (ctypes.c_char * len(data))(*data)
        msg = I2cMsg(addr & 0x7F, 0, len(data), ctypes.addressof(buf))
        rdwr = I2cRdwrIoctlData(ctypes.pointer(msg), 1)
        fcntl.ioctl(fd, I2C_RDWR, rdwr)
    finally:
        fd.close()


class I2cVoicePlayer:
    """Passive broadcast on Yahboom MAE01 / CI1302 via expansion-board I2C."""

    def __init__(
        self,
        *,
        bus: int = 5,
        addr: int = 0x2B,
        logger: Any | None = None,
    ) -> None:
        self._bus = int(bus)
        self._addr = int(addr) & 0x7F
        self._logger = logger

    def play_preset(self, voice_id: int) -> bool:
        """Try CI13 raw packet, then STC8H reg=0x03 single-byte (Yahboom passive table)."""
        vid = int(voice_id) & 0xFF
        packet = bytes(build_ci13_play_packet(vid))
        methods: list[tuple[str, Any]] = [
            ('ci13_raw', lambda: i2c_raw_write(self._bus, self._addr, packet)),
            ('stc8h_reg03', lambda: self._smbus_byte(CI13_PLAY_REG, vid)),
            ('ci13_smbus', lambda: self._smbus_block(CI13_PLAY_REG, packet[1:])),
        ]
        last_exc: Exception | None = None
        for name, fn in methods:
            try:
                fn()
                self._log_info(
                    f'I2C play id=0x{vid:02X} via {name} '
                    f'bus={self._bus} addr=0x{self._addr:02X} pkt={list(packet)}'
                )
                return True
            except OSError as exc:
                last_exc = exc
                self._log_info(f'I2C {name} failed: {exc}')
        if last_exc is not None:
            self._log_error(
                f'I2C voice failed (/dev/i2c-{self._bus} 0x{self._addr:02X}): {last_exc}'
            )
        return False

    def _smbus_byte(self, reg: int, val: int) -> None:
        if smbus2 is None:
            raise OSError('smbus2 not installed')
        with smbus2.SMBus(self._bus) as i2c:
            i2c.write_byte_data(self._addr, reg & 0xFF, val & 0xFF)

    def _smbus_block(self, reg: int, data: bytes | list[int]) -> None:
        if smbus2 is None:
            raise OSError('smbus2 not installed')
        payload = list(data) if isinstance(data, (bytes, bytearray)) else data
        with smbus2.SMBus(self._bus) as i2c:
            i2c.write_i2c_block_data(self._addr, reg & 0xFF, payload)

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[I2cVoicePlayer] {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[I2cVoicePlayer] ERROR: {message}')
