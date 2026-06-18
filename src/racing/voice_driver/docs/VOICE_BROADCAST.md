# Voice Broadcast

**请先读：[VOICE_SETUP.md](VOICE_SETUP.md)** — 针对「只有 MAE01 语音模块」的完整说明。

## 快速命令

```bash
source install/setup.bash

# 模块预设（先喊「小亚小亚」）
ros2 run voice_driver voice_speak forward

# API 文字（.env 里 AUDIO_OUTPUT=mae01 或 alsa）
ros2 run voice_driver voice_speak_text -- "测试"

# 图片 → 大模型 → 播报
ros2 run voice_driver voice_broadcast -- --image /path/to.jpg

# ROS 节点
ros2 launch voice_driver voice_broadcast.launch.py mode:=tts_only
```

## AUDIO_OUTPUT

| 值 | 出声位置 | 能否播 API 任意长文本 |
|----|----------|----------------------|
| `mae01` | MAE01 模块喇叭 | 仅预设/短提示 |
| `alsa` | RDK/USB 音箱 | ✅ 百炼 TTS |
| `both` | 两者都试 | 视硬件而定 |
