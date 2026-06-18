#!/usr/bin/env python3
"""Play Yahboom voice preset on module speaker (I2C + UART)."""

from __future__ import annotations

import argparse
import sys
import time

from voice_api.env_config import VoiceEnvConfig
from voice_api.i2c_player import build_ci13_play_packet
from voice_api.module_player import ModuleVoicePlayer

PRESETS: dict[str, int] = {
    'welcome': 0x00,
    'forward': 0x03,
    'go_ahead': 0x04,
    'stop': 0x11,
    'back': 0x05,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Play Yahboom voice preset on MAE01 module',
    )
    parser.add_argument(
        'preset',
        nargs='?',
        default='forward',
        choices=list(PRESETS.keys()),
        help='Preset name (default: forward)',
    )
    parser.add_argument('--id', type=int, default=-1, help='Raw command ID')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--pause', type=float, default=1.5)
    args = parser.parse_args(argv)

    voice_id = args.id if args.id >= 0 else PRESETS[args.preset]
    pkt = build_ci13_play_packet(voice_id)
    print(f'播报: {args.preset} id=0x{voice_id:02X} CI13包={pkt}')

    player = ModuleVoicePlayer.from_config(VoiceEnvConfig.from_env())
    for i in range(args.repeat):
        if not player.play_preset(voice_id):
            return 1
        print(f'  已发送 ({i + 1}/{args.repeat})')
        if i + 1 < args.repeat:
            time.sleep(args.pause)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
