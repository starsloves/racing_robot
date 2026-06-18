# 语音播报 — 仅 MAE01 语音交互模块

## 你的硬件（当前实际情况）

```
┌─────────────┐   I2C bus5@0x2b    ┌──────────────────┐
│  RDK X5     │ ────────────────── │ YB-MAE01 语音模块 │
│             │   UART /dev/ttyS1  │  麦克风 + 喇叭    │  ← 你唯一能听到的声音
└─────────────┘                    └──────────────────┘
       │
       └── ES8326 板载音频（Origincar 上通常没接喇叭，听不到）
```

| 能力 | 模块自己（喊「小亚小亚」） | 代码触发 |
|------|---------------------------|----------|
| 唤醒 / 口令控车 | ✅ 正常 | — |
| 播 **固件预设** 短句 | ✅ | I2C 或 UART 被动播报（需先唤醒，20 秒内） |
| 播 **API 任意长文本** | ❌ 固件不支持 | 百炼 TTS → 只能播到板载喇叭（你听不到） |

**结论：** 大模型返回的**任意中文**，MAE01 **不能**像智能音箱那样朗读；只能：
1. **预设播报**（如 forward / stop / welcome）— 从模块喇叭出；
2. **百炼 TTS** — 需要 RDK **外接 USB 音箱或耳机** 才能听到。

---

## 一条命令速查

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
cd ~/dev_ws/src/racing/voice_driver   # 读取本目录 .env
```

| 目的 | 命令 |
|------|------|
| 测模块预设（先喊「小亚小亚」） | `ros2 run voice_driver voice_speak forward` |
| 播 API/大模型文字（TTS，需外接音箱） | `ros2 run voice_driver voice_speak_text -- "文字"` |
| 图片→大模型→TTS | `ros2 run voice_driver voice_broadcast -- --image /path/to.jpg` |
| 启动 ROS 自动播报 | `ros2 launch voice_driver voice_broadcast.launch.py mode:=tts_only` |
| 硬件诊断 | `ros2 run voice_driver voice_hardware_test --preset-only` |

---

## `.env` 必改项

路径：`src/racing/voice_driver/.env`

```env
# 只有 MAE01 模块喇叭 → 保持 mae01
AUDIO_OUTPUT=mae01

# 百炼（视觉 + TTS）
DASHSCOPE_API_KEY=sk-...
DASHSCOPE_TTS_MODEL=qwen-tts-realtime-2025-07-15
DASHSCOPE_TTS_VOICE=Cherry

# 模块通信（Origincar 实测）
VOICE_I2C_BUS=5
VOICE_I2C_ADDR=43          # 0x2b
VOICE_SERIAL_PORT=/dev/ttyS1
VOICE_SERIAL_BAUD=115200
# ttyACM0 = 电机 STM32，不是语音模块
```

若以后要听 **任意 TTS 长文本**：外接 USB 音箱，改 `AUDIO_OUTPUT=alsa`。

---

## 推荐数据流

### A. 图片 → 大模型 → 语音（TTS，需外接音箱）

```
/image 或 CLI --image  →  百炼视觉  →  文字  →  qwen-tts-realtime  →  WAV  →  aplay
```

### B. 模块预设（你现在的喇叭）

```
代码  →  I2C 0x2b / UART AA55  →  MAE01 播固件短句（先唤醒）
```

### C. ROS 集成（vision_ai + 播报）

```bash
# 终端1：视觉节点（发布 ai_description）
ros2 run racing_vision_ai vision_ai_node

# 终端2：收到文字后 TTS 播报
ros2 launch voice_driver voice_broadcast.launch.py mode:=tts_only
```

---

## 模块代码播报步骤

1. 对模块说：**「小亚小亚」**，听到 **「我在」**
2. **20 秒内** 运行：`ros2 run voice_driver voice_speak forward`
3. 仍无声：检查四芯 I2C 线是否接 RDK 扩展口；`i2cdetect -y -r 5` 应有 `2b`

---

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select voice_driver
```
