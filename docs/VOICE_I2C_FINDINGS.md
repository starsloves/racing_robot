# MAE01 Voice Module I2C Findings

## 硬件信息

- 模块型号：YB-MAE01-V1.1（亚博 AI 语音交互集成模块）
- 芯片：CI1302 + STC8H 协处理器
- 连接方式：I2C bus 5，地址 0x2B
- 扩展板：Origincar RDK X5

## I2C 协议（已验证可用）

### 播报预设短语

```python
import smbus2

with smbus2.SMBus(5) as i2c:
    i2c.write_i2c_block_data(0x2B, 0x03, [preset_id])
```

- **I2C 地址**：0x2B
- **寄存器**：0x03
- **数据**：单字节 preset_id
- **不需要** checksum 和 0x5A tail byte

### 已验证的预设 ID

| ID | 内容 |
|----|------|
| 0x67 | "这是西瓜皮，属于是垃圾"（已验证出声） |

### 关键发现

1. 之前的 `build_ci13_play_packet()` 生成的 `[0x03, cmd_id, checksum, 0x5A]` 格式**不工作**
2. 正确格式是 **单字节**：`[preset_id]`，不需要 checksum 和 tail
3. CI1302 默认地址 0x64 不可用，Yahboom 模块用 STC8H 转接到 0x2B
4. 所有寄存器读回都是 0xFF（写-only 设备）

### i2c_player.py 需要修正

当前代码的 `ci13_raw` 方法发送 `[0x03, cmd_id, checksum, 0x5A]`，应该改为只发 `[cmd_id]`。

## UART 探测结果

### 连接方式

**Type-C 连接（CH341 USB转串口）：**
- 模块 Type-C → RDK USB 口 → `/dev/ttyUSB0`
- 滑动开关位置决定连接目标：
  - **位置1**：CI1302 原生 UART（固件烧录口）
  - **位置2**：STC8H 协处理器（串口通讯口）

**UART 排针连接（标号11）：**
- 引脚：VCC(5V) / GND / TX1(PA26) / RX1(PA13)
- 接 RDK 40pin：TX1→Pin8(UART1 RX), RX1→Pin10(UART1 TX), GND→Pin6
- 设备：`/dev/ttyS1`（MMIO 0x34070000）
- **结果：无响应，确认不通**

### Type-C UART 测试（ttyUSB0）

| 波特率 | 启动数据 | TTS 响应 | 预设响应 | 声音 |
|--------|----------|----------|----------|------|
| 9600 | `02041cf8` | `004400f9` | AA55 0x67→`40241440ff` | ❌ 无 |
| 115200 | - | - | - | ❌ 无 |

- FD TTS 格式：模块有响应但不出声
- Yahboom AA55 预设格式：部分 ID 有响应但不出声
- V2 play local broadcast：部分 ID 有响应但不出声

### 关键结论

1. **CI1302 TTS 需要 license**（启英泰伦收费），Yahboom 出厂固件可能未烧 TTS license
2. **Type-C 在位置1时是固件烧录口**，不是 TTS 数据口
3. **STC8H UART（标号11）只接收命令不发送数据**
4. **模块喇叭正常**（唤醒词"小亚小亚"→"我在"能出声）

## 任意文字播报方案

当前硬件条件下，MAE01 模块**无法通过串口实现任意文字 TTS**。可行方案：

1. **I2C 预设播报** → 扫描预设 ID，选最接近的短语（已验证出声）
2. **重烧 TTS 固件** → 需要启英泰伦平台 + TTS license + 烧录工具
3. **接喇叭到 RDK 音频口** → 用云 TTS（DashScope），软件链路已验证通过

## 参考资料

- 启英泰伦 CI13XX I2C 协议：https://document.chipintelli.com/
- 启英泰伦 CI1302 UART 协议 V2：https://document.chipintelli.com/
- 亚博语音模块教程：https://www.yahboom.com/study/Voice-ASR-TTS
- 模块支持自定义播报内容（官方回复确认，但需重烧固件）
