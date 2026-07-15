"""Drive voice modules: CN-TTS (新，支持自定义文本) 和 Yahboom MAE01 (旧，仅预设短句)."""

from __future__ import annotations

import re
from typing import Any

from .cn_tts_player import CnTtsPlayer
from .env_config import VoiceEnvConfig
from .i2c_player import I2cVoicePlayer


# 关键词 → 友好语音提示（用于 CN-TTS）
_TEXT_HINTS: list[tuple[re.Pattern[str], str]] = [
    (re.compile(r'停|停止|stop', re.I), '停止'),
    (re.compile(r'前进|向前|go\s*ahead|forward', re.I), '前进'),
    (re.compile(r'后退|back', re.I), '后退'),
    (re.compile(r'左转|左', re.I), '左转'),
    (re.compile(r'右转|右', re.I), '右转'),
]


class ModuleVoicePlayer:
    """语音模块播放器（优先使用 CN-TTS，支持任意文本朗读）"""

    FALLBACK_PORTS = ('/dev/ttyS1', '/dev/ttyS5')
    FALLBACK_BAUDS = (9600, 115200)

    def __init__(
        self,
        *,
        cn_tts: CnTtsPlayer,
        i2c: I2cVoicePlayer | None = None,
        logger: Any | None = None,
    ) -> None:
        """
        初始化语音模块播放器
        
        Args:
            cn_tts: CN-TTS UART 播放器（支持自定义文本）
            i2c: I2C 播放器（可选，用于 MAE01 等旧模块）
            logger: ROS 2 logger
        """
        self._cn_tts = cn_tts
        self._i2c = i2c
        self._logger = logger

    @classmethod
    def from_config(
        cls,
        config: VoiceEnvConfig | None = None,
        *,
        logger: Any | None = None,
        print_wake_hint: bool = True,  # 兼容旧接口，已废弃
    ) -> ModuleVoicePlayer:
        """从环境配置创建播放器"""
        cfg = config or VoiceEnvConfig.from_env()
        
        # 优先使用 CN-TTS（新模块，支持自定义文本）
        cn_tts = CnTtsPlayer(
            port=cfg.voice_serial_port or '/dev/ttyS1',
            baudrate=9600,  # CN-TTS 标准波特率
            logger=logger,
        )
        
        # 可选：保留 I2C 兼容性（用于旧的 MAE01 模块）
        i2c = I2cVoicePlayer(
            bus=cfg.voice_i2c_bus,
            addr=cfg.voice_i2c_addr,
            logger=logger,
        ) if cfg.voice_i2c_bus > 0 else None
        
        return cls(
            cn_tts=cn_tts,
            i2c=i2c,
            logger=logger,
        )

    def speak_text(self, text: str) -> bool:
        """
        播报自定义文本（CN-TTS 支持任意中文/英文/数字）
        
        Args:
            text: 要播报的文本
            
        Returns:
            bool: 播报是否成功
        """
        cleaned = text.strip()
        if not cleaned:
            return False

        # CN-TTS 支持自定义文本朗读，直接发送
        self._log_info(f'CN-TTS 播报文本 (长度={len(cleaned)}): {cleaned[:100]}...')
        
        # 1) 尝试主端口
        if self._cn_tts.speak_text(cleaned):
            return True
        
        # 2) 尝试备用端口（如果主端口失败）
        for port in self.FALLBACK_PORTS:
            if port == self._cn_tts._port:  # noqa: SLF001
                continue
            for baud in self.FALLBACK_BAUDS:
                backup_player = CnTtsPlayer(port=port, baudrate=baud, logger=self._logger)
                if backup_player.speak_text(cleaned):
                    self._log_info(f'备用端口成功: {port}@{baud}')
                    return True
        
        self._log_error('CN-TTS 播报失败，请检查串口连接和权限')
        return False

    def set_volume(self, level: int) -> bool:
        """设置音量等级 (1-4)"""
        return self._cn_tts.set_volume(level)

    def set_speed(self, level: int) -> bool:
        """设置语速等级 (1-3)"""
        return self._cn_tts.set_speed(level)

    def play_sound_effect(self, effect_id: int) -> bool:
        """播放音效 (0-7)"""
        return self._cn_tts.play_sound_effect(effect_id)

    def play_preset(self, voice_id: int) -> bool:
        """
        播放预设短语（兼容旧接口）
        新 CN-TTS 模块通过音效 ID 播放，或直接文本播报
        """
        # 将预设 ID 映射到音效或文本
        preset_map = {
            0x00: '欢迎',
            0x03: '前进',
            0x05: '后退',
            0x0F: '左转',
            0x10: '右转',
            0x11: '停止',
        }
        
        text = preset_map.get(voice_id)
        if text:
            return self.speak_text(text)
        
        # 未知 ID，尝试音效
        if 0 <= voice_id <= 7:
            return self.play_sound_effect(voice_id)
        
        self._log_error(f'未知预设 ID: 0x{voice_id:02X}')
        return False

    def play_named(self, name: str) -> bool:
        """
        播放命名预设（兼容旧接口）
        CN-TTS 支持直接文本播报
        """
        key = name.strip().lower()
        
        # 关键词映射
        keyword_map = {
            'welcome': '欢迎',
            'forward': '前进',
            'back': '后退',
            'left': '左转',
            'right': '右转',
            'stop': '停止',
        }
        
        text = keyword_map.get(key, name)
        return self.speak_text(text)

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
