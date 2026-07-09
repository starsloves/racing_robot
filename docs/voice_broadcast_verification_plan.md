# 语音模块自定义播报 — 验证规划与执行记录

> 目标：验证 `racing_vision_ai`（图片识别 API）返回的 `ai_description` 文本能被 `voice_driver` 真正朗读出来。
> 修改已落地：`voice_broadcast.py` 云端 TTS 优先 + `.env` `AUDIO_OUTPUT=both`、`BROADCAST_MODE=tts_only`。
> 测试在远端板子（100.114.34.86）执行，本机只写文档与下发命令。

## 链路回顾
```
相机 /image(CompressedImage)
   └─ racing_vision_ai: 收 sign4return=9 → 抓一帧 → 调识别API → 发布 /ai_description
        └─ voice_driver(tts_only): 订阅 /ai_description → speak_text() → 云端TTS(优先) → 音箱
                                                                    └─ 失败则 MAE01 兜底
```

## 验证阶段

### 阶段 0 — 环境确认（板上，已部分完成）
- [x] voice_driver / racing_vision_ai 已编译
- [x] .env: AUDIO_OUTPUT=both, BROADCAST_MODE=tts_only
- [ ] aplay -l 确认音箱/板载音频设备
- [ ] racing_vision_ai 的 vision_ai_config.yaml 有 Volcengine api_key/model

### 阶段 1 — 仅语音（不依赖摄像头）  ← 当前执行
1. 远端后台启动 `ros2 launch voice_driver voice_tts.launch.py`（tts_only）
2. 远端 `ros2 topic pub /ai_description` 灌入测试文本
3. 监听 `ros2 topic echo /voice_broadcast_status`
4. 用户听声确认
- 判定：`voice_broadcast_status` 出现 `text_ok` 且听到人声 → 语音链路 OK
- 失败排查：AUDIO_DEVICE、DASHSCOPE_API_KEY、网络、音箱

### 阶段 2 — 仅识别产出（依赖摄像头）
1. 起相机 `ros2 launch origincar_bringup usb_websocket_display.launch.py`
2. `ros2 topic echo /image` 确认有 CompressedImage
3. `ros2 run racing_vision_ai vision_ai_node`
4. `ros2 topic pub /sign4return std_msgs/msg/Int32 "{data: 9}"`
5. `ros2 topic echo /ai_description` 看文本
- 判定：ai_description 出现文字 → 识别链路 OK

### 阶段 3 — 端到端
同时跑阶段1语音节点 + 阶段2相机/识别，发 sign4return=9 → 自动播报识别结果。

### 阶段 4 — 边界
- 长文本完整播报（验证云 TTS 优先）
- 断网回退 MAE01 提示音（验证 both 兜底）

## 执行记录
- 2026-07-08: 代码改动 + 远端编译通过；开始阶段1。
- 阶段1 结果（软件侧通过）：远端起 voice_tts.launch.py(tts_only)，灌入两段 ai_description
  （"测试语音播报，前方红色锥桶" 13字 / "test broadcast red cone" 23字），
  日志确认 AUDIO_OUTPUT=both → dashscope TTS 合成 wav(169KB/122KB) → aplay -D plughw:0,0 →
  voice_broadcast_status: text_ok。需用户确认实际听到人声。
- 待办：用户确认听感后进入阶段2（相机 /image + racing_vision_ai 产出 ai_description）。

## 关键结论（联网核实 + 板端实测）：MAE01 不能朗读任意文本
- YB-MAE01-V1.1 = CI1302 方案，I2C 从机 addr 0x2b（i2cdetect bus5 已确认）。
- 板端实测：ttyS1/ttyS5 串口帧无反应（模块 UART 未接 RDK）；I2C bus5/0x2b 能写成功但
  预设/“welcome”均不发声（模块可能休眠或 I2C 转发芯片未接 MAE01 喇叭）。
- CI1302 的 I2C/串口都只能按 ID 播预录短语，不能接收任意文本做 TTS（Yahboom 文档 +
  yahboom-mcp 项目均确认：任意文本 TTS 必须放在主机，走 ES8326 音频口）。
- 代码 mae01_player 的 SYN6288(0xFD) 帧是另一种 TTS 芯片协议，对 CI1302/I2C 无效。

## 自定义文本播报的可行路径（当前硬件）
- 路线A（推荐，软件已验证）：主机云端 TTS → ES8326(duplex-audio) → 板载/USB 音箱。
  已验证 aplay 成功跑通，唯一缺「音频口接喇叭」。
- 路线B：把 MAE01 的 UART 引到 RDK 空闲串口 + 用 0xFD TTS 帧（需改接线/硬件）。
- 路线C：换 XFS5152CE 类真·TTS 模块（I2C/UART 收任意文本）。
- MAE01 现状：只能作预置提示音（若 I2C 能调通发声），不能朗读 API 原文。
