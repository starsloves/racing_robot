#!/usr/bin/env python3
"""One-shot hardware + TTS test after configuring .env."""

from __future__ import annotations

import argparse
import sys

from voice_api.env_config import VoiceEnvConfig
from voice_api.module_player import ModuleVoicePlayer
from voice_api.voice_broadcast import VoiceBroadcastService
from voice_api.voice_ids import VoiceId


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Test voice hardware and cloud TTS')
    parser.add_argument('--tts-only', action='store_true', help='Only cloud TTS + ALSA')
    parser.add_argument('--preset-only', action='store_true', help='Only MAE01/I2C presets')
    args = parser.parse_args(argv)

    config = VoiceEnvConfig.from_env()
    print(f'API key loaded: {bool(config.dashscope_api_key)}')
    print(f'TTS provider: {config.resolved_tts_provider()}')
    print(f'Audio output: {config.audio_output} device={config.audio_device}')
    print(f'Voice UART: {config.voice_serial_port} (do NOT use ttyACM0=motor)')

    if not args.preset_only:
        print('\n--- Cloud TTS (百炼) -> board ALSA ---')
        ok = VoiceBroadcastService(config).speak_text('百炼语音测试，如果你听到这段话说明云端播报正常。')
        print('TTS result:', 'ok' if ok else 'FAILED')

    if not args.tts_only:
        print('\n--- MAE01 module preset (forward id=0x03) ---')
        module = ModuleVoicePlayer.from_config(config)
        ok_mod = module.play_named('forward')
        print('Module preset:', 'sent' if ok_mod else 'failed (check I2C/UART wiring)')

        print('\n--- MAE01 module preset (go_ahead id=4) ---')
        ok_mod2 = module.play_preset(VoiceId.GO_AHEAD)
        print('go_ahead:', 'sent' if ok_mod2 else 'failed')

    print('\nIf all silent: MAE01 speaker is on the module; ALSA is board codec — check wiring/power.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
