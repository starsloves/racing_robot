#!/usr/bin/env python3
"""CN-TTS 模块测试命令行工具"""

import sys
import argparse
from cn_tts_player import CnTtsPlayer


def main():
    parser = argparse.ArgumentParser(description='CN-TTS 语音模块测试工具')
    parser.add_argument('--port', default='/dev/ttyS1', help='串口设备路径（默认 /dev/ttyS1）')
    parser.add_argument('--baud', type=int, default=9600, help='波特率（默认 9600）')
    
    subparsers = parser.add_subparsers(dest='command', help='命令')
    
    # 播报文本
    speak_parser = subparsers.add_parser('speak', help='播报自定义文本')
    speak_parser.add_argument('text', nargs='+', help='要播报的文本')
    
    # 设置音量
    volume_parser = subparsers.add_parser('volume', help='设置音量 (1-4)')
    volume_parser.add_argument('level', type=int, choices=[1, 2, 3, 4], help='音量等级')
    
    # 设置语速
    speed_parser = subparsers.add_parser('speed', help='设置语速 (1-3)')
    speed_parser.add_argument('level', type=int, choices=[1, 2, 3], help='语速等级')
    
    # 播放音效
    effect_parser = subparsers.add_parser('effect', help='播放音效 (0-7)')
    effect_parser.add_argument('id', type=int, choices=range(8), help='音效编号')
    
    # 测试预设短语
    preset_parser = subparsers.add_parser('preset', help='测试预设短语')
    preset_parser.add_argument('name', choices=['欢迎', '前进', '后退', '左转', '右转', '停止'])
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # 创建 CN-TTS 播放器
    player = CnTtsPlayer(port=args.port, baudrate=args.baud)
    
    # 执行命令
    if args.command == 'speak':
        text = ' '.join(args.text)
        print(f'播报文本: {text}')
        success = player.speak_text(text)
    elif args.command == 'volume':
        print(f'设置音量: {args.level}')
        success = player.set_volume(args.level)
    elif args.command == 'speed':
        print(f'设置语速: {args.level}')
        success = player.set_speed(args.level)
    elif args.command == 'effect':
        print(f'播放音效: {args.id}')
        success = player.play_sound_effect(args.id)
    elif args.command == 'preset':
        print(f'测试预设: {args.name}')
        success = player.speak_text(args.name)
    else:
        print(f'未知命令: {args.command}')
        return 1
    
    if success:
        print('✓ 执行成功')
        return 0
    else:
        print('✗ 执行失败')
        return 1


if __name__ == '__main__':
    sys.exit(main())
