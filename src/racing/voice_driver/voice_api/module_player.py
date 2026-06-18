"""Drive Yahboom MAE01 module speaker (I2C + UART)."""

from __future__ import annotations

import re
from typing import Any

from .env_config import VoiceEnvConfig
from .i2c_player import I2cVoicePlayer
from .mae01_player import Mae01Player, YAHBOOM_ACTIVE_IDS


# 大模型短回复 → 模块预设 ID（仅固件里有的短句）
_TEXT_PRESET_HINTS: list[tuple[re.Pattern[str], str]] = [
    (re.compile(r'停|停止|stop', re.I), 'stop'),
    (re.compile(r'前进|向前|go\s*ahead|forward', re.I), 'forward'),
    (re.compile(r'后退|back', re.I), 'back'),
    (re.compile(r'左', re.I), 'left'),
    (re.compile(r'右', re.I), 'right'),
    (re.compile(r'红', re.I), 'forward'),  # 无颜色预设时用 forward 作提示
]


class ModuleVoicePlayer:
    """Play on YB-MAE01 — the only speaker on Origincar without extra wiring."""

    WAKE_HINT = (
        '【MAE01】请先对模块说「小亚小亚」，听到「我在」后 20 秒内再触发代码。'
    )

    FALLBACK_PORTS = ('/dev/ttyS1', '/dev/ttyS5')
    FALLBACK_BAUDS = (115200, 9600)

    def __init__(
        self,
        *,
        i2c: I2cVoicePlayer,
        uart: Mae01Player,
        logger: Any | None = None,
        print_wake_hint: bool = True,
    ) -> None:
        self._i2c = i2c
        self._uart = uart
        self._logger = logger
        self._print_wake_hint = print_wake_hint

    @classmethod
    def from_config(
        cls,
        config: VoiceEnvConfig | None = None,
        *,
        logger: Any | None = None,
        print_wake_hint: bool = True,
    ) -> ModuleVoicePlayer:
        cfg = config or VoiceEnvConfig.from_env()
        return cls(
            i2c=I2cVoicePlayer(
                bus=cfg.voice_i2c_bus,
                addr=cfg.voice_i2c_addr,
                logger=logger,
            ),
            uart=Mae01Player(
                port=cfg.voice_serial_port,
                baudrate=cfg.voice_serial_baud,
                protocol=cfg.mae01_protocol,
                logger=logger,
            ),
            logger=logger,
            print_wake_hint=print_wake_hint,
        )

    def speak_text(self, text: str) -> bool:
        """Try to play API text on module (limited by MAE01 firmware)."""
        cleaned = text.strip()
        if not cleaned:
            return False

        if self._print_wake_hint:
            print(f'[ModuleVoicePlayer] {self.WAKE_HINT}')

        # 1) 关键词 → 固件预设
        for pattern, name in _TEXT_PRESET_HINTS:
            if pattern.search(cleaned):
                self._log_info(f'Text matched preset "{name}", playing on module')
                if self.play_named(name):
                    print(f'>>> 模块已播预设「{name}」（非完整 API 原文）')
                    return True

        # 2) 多端口 UART 文本协议（SYN6288 / AA55 / $Axxx#）
        ports = []
        for port in (self._uart._port, *self.FALLBACK_PORTS):  # noqa: SLF001
            if port not in ports and port not in Mae01Player.MOTOR_PORTS:
                ports.append(port)

        for port in ports:
            for baud in self.FALLBACK_BAUDS:
                player = Mae01Player(
                    port=port,
                    baudrate=baud,
                    protocol=self._uart._protocol,  # noqa: SLF001
                    logger=self._logger,
                )
                if player.speak_text(cleaned):
                    print(f'>>> 模块 UART 已发送文本 ({port}@{baud})')
                    return True

        # 3) 短文本：播 welcome 作收到提示
        if len(cleaned) <= 40:
            self._log_info('Arbitrary text unsupported; playing welcome ack on module')
            if self.play_preset(0x00):
                print('>>> 模块已播「welcome」提示音（API 全文请见终端/话题）')
                print(f'    原文: {cleaned[:200]}')
                return True

        self._log_error(
            'MAE01 不能朗读任意长文本。改 AUDIO_OUTPUT=alsa 并外接 USB 音箱，'
            '或先唤醒后: ros2 run voice_driver voice_speak forward'
        )
        print(f'【API 文字未出声】{cleaned[:300]}')
        return False

    def play_preset(self, voice_id: int) -> bool:
        if self._print_wake_hint:
            print(f'[ModuleVoicePlayer] {self.WAKE_HINT}')

        vid = int(voice_id) & 0xFF
        ok_i2c = self._i2c.play_preset(vid)
        ok_uart_active = self._uart.play_preset(vid)
        ok_uart_passive = self._uart.play_passive(vid)
        ok_dollar = self._uart.play_dollar(vid)
        ok = ok_i2c or ok_uart_active or ok_uart_passive or ok_dollar
        if ok:
            self._log_info(
                f'Module preset id=0x{vid:02X}: '
                f'i2c={ok_i2c} uart={ok_uart_active}/{ok_uart_passive} '
                f'dollar={ok_dollar}'
            )
        else:
            self._log_error(
                'Module preset failed. Check I2C bus5@0x2b and UART (not ttyACM0).'
            )
        return ok

    def play_named(self, name: str) -> bool:
        key = name.strip().lower()
        vid = YAHBOOM_ACTIVE_IDS.get(key)
        if vid is None:
            from .voice_ids import voice_id_for_name

            resolved = voice_id_for_name(key)
            if resolved is None:
                self._log_error(f'Unknown preset: {name}')
                return False
            vid = resolved
        return self.play_preset(vid)

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[ModuleVoicePlayer] {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[ModuleVoicePlayer] ERROR: {message}')
