"""Play speech on Yahboom YB-MAE01 via UART."""

from __future__ import annotations

import time
from typing import Any

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


# Yahboom CI1302 active command table (AA 55 00 ID FB)
YAHBOOM_ACTIVE_IDS = {
    'welcome': 0x00,
    'forward': 0x03,
    'back': 0x05,
    'left': 0x0F,
    'right': 0x10,
    'stop': 0x11,
}


class Mae01Player:
    """Drive YB-MAE01 speaker through UART text synthesis or preset broadcast."""

    MOTOR_PORTS = frozenset({'/dev/ttyACM0', '/dev/ttyACM1'})

    def __init__(
        self,
        *,
        port: str = '/dev/ttyS1',
        baudrate: int = 115200,
        protocol: str = 'auto',
        logger: Any | None = None,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._protocol = protocol.lower()
        self._logger = logger
        if port in self.MOTOR_PORTS:
            self._log_error(
                f'{port} is Origincar motor STM32 serial; use voice module UART (e.g. /dev/ttyS1)'
            )

    def speak_text(self, text: str) -> bool:
        cleaned = text.strip()
        if not cleaned:
            self._log_error('MAE01 skipped: empty text')
            return False
        if len(cleaned) > 180:
            cleaned = cleaned[:180]

        protocol = self._protocol
        if protocol == 'auto':
            if self._speak_syn6288(cleaned):
                return True
            for frame in (
                bytes([0xAA, 0x55, 0xFF, 0x03, 0xFB]),
                bytes([0xAA, 0x55, 0x01, 0x00, 0xFB]),
            ):
                if self._write_serial(frame, label='auto-fallback'):
                    return True
            return False

        if protocol == 'syn6288':
            return self._speak_syn6288(cleaned)
        if protocol in {'yahboom', 'active', 'passive'}:
            return self.play_welcome()
        self._log_error(f'Unsupported MAE01 protocol: {protocol}')
        return False

    def play_welcome(self) -> bool:
        """Passive welcome phrase: AA 55 01 00 FB."""
        return self._write_serial(bytes([0xAA, 0x55, 0x01, 0x00, 0xFB]), label='welcome')

    def play_preset(self, broadcast_id: int) -> bool:
        """Active preset phrase: AA 55 00 ID FB."""
        voice_id = int(broadcast_id) & 0xFF
        frame = bytes([0xAA, 0x55, 0x00, voice_id, 0xFB])
        return self._write_serial(frame, label=f'active id=0x{voice_id:02X}')

    def play_passive(self, broadcast_id: int) -> bool:
        """Passive preset phrase: AA 55 01 ID FB."""
        voice_id = int(broadcast_id) & 0xFF
        frame = bytes([0xAA, 0x55, 0x01, voice_id, 0xFB])
        return self._write_serial(frame, label=f'passive id=0x{voice_id:02X}')

    def play_dollar(self, cmd_id: int) -> bool:
        """Legacy Yahboom host trigger: $A004# style (some firmware builds)."""
        cid = int(cmd_id) & 0xFF
        # Map low IDs to table: forward=4, stop=2, etc.
        table = {0x03: 4, 0x11: 2, 0x05: 5, 0x0F: 6, 0x10: 7, 0x00: 4}
        num = table.get(cid, cid if cid < 100 else cid % 100)
        payload = f'$A{num:03d}#'.encode('ascii')
        return self._write_serial(payload, label=f'dollar A{num:03d}')

    def play_named(self, name: str) -> bool:
        key = name.strip().lower()
        voice_id = YAHBOOM_ACTIVE_IDS.get(key)
        if voice_id is None:
            self._log_error(f'Unknown MAE01 preset: {name}')
            return False
        return self.play_preset(voice_id)

    def _speak_syn6288(self, text: str) -> bool:
        frame = self._build_syn6288_frame(text)
        ok = self._write_serial(frame, label=f'SYN6288 ({len(text)} chars)')
        if ok:
            time.sleep(min(8.0, 0.08 * len(text) + 1.0))
        return ok

    @staticmethod
    def _build_syn6288_frame(text: str, *, encoding: str = 'gbk') -> bytes:
        text_bytes = text.encode(encoding, errors='ignore')
        data_len = len(text_bytes) + 2
        frame = bytearray([
            0xFD,
            (data_len >> 8) & 0xFF,
            data_len & 0xFF,
            0x01,
            0x01,
        ])
        frame.extend(text_bytes)
        checksum = 0
        for value in frame:
            checksum ^= value
        frame.append(checksum)
        return bytes(frame)

    def _write_serial(self, payload: bytes, *, label: str) -> bool:
        if self._port in self.MOTOR_PORTS:
            return False
        if serial is None:
            self._log_error('pyserial not installed')
            return False
        try:
            with serial.Serial(
                self._port,
                self._baudrate,
                timeout=1,
                write_timeout=2,
            ) as ser:
                ser.reset_input_buffer()
                ser.write(payload)
                ser.flush()
            hex_preview = payload.hex(' ')
            self._log_info(f'MAE01 {label} on {self._port}@{self._baudrate}: {hex_preview}')
            return True
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'MAE01 serial write failed ({self._port}): {exc}')
            return False

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[Mae01Player] {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[Mae01Player] ERROR: {message}')
