# audio_input

ROS2 音频输入包，用于本地音频录制和语音识别。

## 功能描述

该包实现了一个 ROS2 节点，提供以下功能：

- **音频录制**：使用 PyAudio 进行本地音频录制
- **语音活动检测 (VAD)**：使用 Silero VAD 模型检测语音的开始和结束
- **语音识别 (ASR)**：使用 FunASR (Paraformer-large 模型) 进行中文语音识别
- **唤醒词检测**：支持配置唤醒词
- **状态管理**：与 LLM 状态机集成，支持 listening/input_processing 等状态

## 依赖

- `llm_config` - LLM 配置包
- `rclpy` - ROS2 Python 客户端库
- `std_msgs` - 标准消息包
- `pyaudio` - Python 音频库
- `sounddevice` - 音频设备库
- `numpy` - 数值计算库
- `funasr` - 阿里 FunASR 语音识别
- `scipy` - 科学计算库

## 启动方式

### 方式 1：使用 ROS2 运行

```bash
# 启动音频输入节点
ros2 run audio_input audio_input_local.py

# 监听识别结果
ros2 topic echo /audio_input_audio_to_text

# 触发监听（发布 listening 状态）
ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
```

### 方式 2：订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/llm_state` | std_msgs/String | LLM 状态，发布 "listening" 开始录音 |
| `/audio_input_audio_to_text` | std_msgs/String | 语音识别结果输出 |
| `/llm_initialization_state` | std_msgs/String | 节点初始化状态 |

## 关键文件

```
audio_input/
├── audio_input/
│   ├── audio_input_local.py    # 主节点实现
│   ├── vad.py                  # 语音活动检测模块
│   ├── vad_record_pyaudio.py   # PyAudio VAD 录制
│   └── funsar_rec.py           # FunASR 识别接口
├── resource/
│   ├── user_audio_input.wav    # 临时音频文件
│   ├── silero_vad.onnx         # VAD 模型
│   ├── ding-126626.mp3         # 提示音
│   └── hotword.txt             # 唤醒词配置
├── setup.py                    # Python 包配置
└── package.xml                 # 包清单
```

## 工作流程

1. 节点启动后订阅 `/llm_state` 话题
2. 收到 `listening` 状态时，播放提示音开始录音
3. 使用 VAD 检测语音开始和结束
4. 录音完成后，使用 FunASR 进行语音识别
5. 识别结果发布到 `/audio_input_audio_to_text` 话题
6. 状态更新为 `input_processing`

## 配置

- 模型路径：`iic/speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch`
- 采样率：16000 Hz
- 音频通道：1 (单声道)
- VAD 模型：Silero VAD

---
Maintainer: Herman Ye (hermanye233@icloud.com)
License: Apache-2.0
