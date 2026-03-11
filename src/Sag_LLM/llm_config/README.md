# llm_config

LLM 配置功能包，提供大语言模型 API 配置和系统提示词管理。

## 功能描述

该包是 ROS-LLM 系统的配置模块，主要功能包括：

- **API 配置管理**: 配置 Qwen (DashScope) API 密钥和模型参数
- **系统提示词**: 加载和管理机器人的系统提示词（System Prompt）
- **对话历史**: 管理聊天历史记录，支持 JSON 文件持久化
- **音频配置**: 配置音频录制参数（采样率、时长、VAD 阈值）
- **ROS 话题配置**: 定义 Realsense 相机话题等

## 依赖

- `ament_index_python` - 用于获取包共享目录

## 主要配置项

### LLM API 配置
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `dashscope_api_key` | DashScope API 密钥 | - |
| `qwen_llm_model` | LLM 模型名称 | `qwen-turbo` |
| `qwen_vlm_model` | VLM 模型名称 | `qwen-vl-plus` |
| `llm_temperature` | 生成温度 (0-2) | 1 |
| `llm_top_p` | Top P 采样 | 1 |
| `llm_max_tokens` | 最大生成 Token 数 | 4000 |

### 话题配置
| 参数 | 说明 |
|------|------|
| `rs_color_image_topic` | Realsense 彩色图像话题 |
| `rs_depth_image_topic` | Realsense 深度图像话题 |
| `rs_camera_info_topic` | Realsense 相机信息话题 |

## 关键文件

| 文件 | 说明 |
|------|------|
| `llm_config/user_config.py` | 主配置文件，包含 `UserConfig` 类 |
| `resource/llm_prompt.md` | 简短系统提示词 |
| `resource/llm_prompt_full.md` | 完整系统提示词 |

## 使用方法

```python
from llm_config.user_config import UserConfig

# 初始化配置
config = UserConfig()

# 访问配置项
api_key = config.dashscope_api_key
model = config.qwen_llm_model
system_prompt = config.system_prompt
```

## 注意事项

- `dashscope_api_key` 可通过环境变量 `DASHSCOPE_API_KEY` 设置，也可在代码中硬编码
- 聊天历史默认保存到用户主目录，文件名为 `chat_history_YYYY-MM-DD-HH-MM-SS.json`
