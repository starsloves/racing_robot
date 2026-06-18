#!/usr/bin/env python3
"""Sweep serial/I2C/ALSA paths to find working voice hardware."""

from __future__ import annotations

import argparse
import glob
import subprocess
import sys
import time
from pathlib import Path

from voice_api.i2c_player import I2cVoicePlayer
from voice_api.mae01_player import Mae01Player


def list_serial_ports() -> list[str]:
    ports = sorted(glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyACM*'))
    return [port for port in ports if Path(port).exists()]


def probe_serial(ports: list[str], bauds: list[int]) -> None:
    print('\n=== Serial MAE01 (Yahboom AA 55 00 03 FB = forward) ===')
    for port in ports:
        for baud in bauds:
            player = Mae01Player(port=port, baudrate=baud, protocol='yahboom')
            ok = player.play_preset(0x03)
            print(f'  {port} @ {baud}: {"write ok" if ok else "failed"}')
            time.sleep(1.2)
        player = Mae01Player(port=port, baudrate=bauds[0], protocol='yahboom')
        player.play_welcome()
        print(f'  {port} welcome frame sent')
        time.sleep(1.2)


def probe_i2c(buses: list[int], addrs: list[int]) -> None:
    print('\n=== I2C preset (reg 0x03, id=0x03 forward) ===')
    for bus in buses:
        for addr in addrs:
            player = I2cVoicePlayer(bus=bus, addr=addr)
            ok = player.play_preset(0x03)
            print(f'  i2c-{bus} addr=0x{addr:02X}: {"ok" if ok else "failed"}')
            time.sleep(0.8)


def probe_alsa(device: str) -> None:
    print(f'\n=== ALSA beep on {device} ===')
    wav = Path('/tmp/voice_probe_beep.wav')
    try:
        subprocess.run(
            [
                'ffmpeg', '-y', '-f', 'lavfi', '-i', 'sine=frequency=880:duration=1',
                str(wav),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        subprocess.run(['amixer', '-c', '0', 'set', 'SPKVol', '100%'], check=False)
        subprocess.run(['aplay', '-q', '-D', device, str(wav)], check=True)
        print('  aplay beep sent')
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f'  ALSA probe failed: {exc}')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Probe Yahboom voice hardware paths')
    parser.add_argument('--serial-only', action='store_true')
    parser.add_argument('--i2c-only', action='store_true')
    parser.add_argument('--alsa-only', action='store_true')
    parser.add_argument('--device', default='plughw:0,0')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print('Voice hardware probe — listen for speech/beep on the robot speaker.')
    if not args.i2c_only and not args.alsa_only:
        probe_serial(list_serial_ports(), [115200, 9600])
    if not args.serial_only and not args.alsa_only:
        probe_i2c(list(range(0, 8)), [0x2B, 0x43, 0x64, 0x30])
    if not args.serial_only and not args.i2c_only:
        probe_alsa(args.device)
    print('\nDone. Tell the agent which step produced sound.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
