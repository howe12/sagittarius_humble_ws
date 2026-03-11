# llm_model

LLM 模型功能包，提供基于大语言模型的对话机器人和函数调用功能。

## 功能描述

该包是 ROS-LLM 系统的核心节点，实现了：

- **对话交互**: 接收用户文本输入，与 LLM 进行对话
- **函数调用**: 解析 LLM 返回的函数调用请求，调用机器人功能服务
- **聊天历史管理**: 维护对话上下文，支持历史记录持久化到 JSON
- **多模型支持**: 支持 Qwen (DashScope) API 和本地 MiniCPM3 模型
- **响应分发**: 根据 LLM 响应类型（文本/函数调用）执行不同逻辑

## 依赖

### Python 依赖
- `llm_config` - 配置模块
- `llm_interfaces` - 接口定义
- `requests` - HTTP 请求库

### 系统依赖
- `rclpy` - ROS2 Python 客户端库

## 启动方式

### 启动 LLM 节点
```bash
ros2 run llm_model chatllm
```

### 测试节点

1. 启动节点：
```bash
ros2 run llm_model chatllm
```

2. 订阅响应话题：
```bash
ros2 topic echo /llm_feedback_to_user
```

3. 发布输入话题：
```bash
ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'Hello, tell me a joke'" -1
```

## ROS 话题

### 订阅话题 (Subscriptions)
| 话题 | 类型 | 说明 |
|------|------|------|
| `/llm_input_audio_to_text` | `std_msgs/String` | 用户输入（文本） |
| `/llm_state` | `std_msgs/String` | LLM 状态 |

### 发布话题 (Publications)
| 话题 | 类型 | 说明 |
|------|------|------|
| `/llm_initialization_state` | `std_msgs/String` | 初始化状态 |
| `/llm_state` | `std_msgs/String` | LLM 状态 |
| `/llm_response_type` | `std_msgs/String` | 响应类型 |
| `/llm_feedback_to_user` | `std_msgs/String` | 对用户反馈 |
| `/vlm_feedback_to_user` | `std_msgs/String` | VLM 反馈 |
| `/ChatLLM_text_output` | `std_msgs/String` | 文本输出 |

### 服务客户端 (Service Clients)
| 服务 | 类型 | 说明 |
|------|------|------|
| `/ChatLLM_function_call_service` | `llm_interfaces/srv/ChatLLM` | LLM 函数调用 |
| `/MotionRobot_function_call_service` | `llm_interfaces/srv/ChatLLM` | VLM 函数调用 |

## 关键文件

| 文件 | 说明 |
|------|------|
| `llm_model/chatLLM.py` | 主节点代码，包含 `ChatLLMNode` 类 |

## 配置

LLM 模型参数在 `llm_config` 包中配置：

```python
# llm_config/user_config.py
self.dashscope_api_key = "sk-..."        # API 密钥
self.qwen_llm_model = "qwen-turbo"       # 模型名称
self.llm_temperature = 1                  # 温度
self.llm_max_tokens = 4000               # 最大 Token 数
```

## 工作流程

1. 节点启动 → 初始化发布者/订阅者/服务客户端
2. 接收用户输入 `/llm_input_audio_to_text`
3. 添加到聊天历史
4. 调用 LLM 生成响应
5. 解析响应：
   - **文本响应**: 直接发布到 `/llm_feedback_to_user`
   - **函数调用**: 调用 `/ChatLLM_function_call_service` 或 `/MotionRobot_function_call_service`
6. 更新聊天历史并保存到 JSON 文件
