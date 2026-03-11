# llm_interfaces

ROS-LLM 接口功能包，定义机器人与大语言模型交互的 ROS 消息、服务和动作接口。

## 功能描述

该包提供 ROS-LLM 系统的标准接口定义，用于：
- 聊天对话交互
- 机械臂关节控制
- 机械臂末端位置/姿态控制
- 视觉识别任务
- 抓取和放置任务

## 依赖

### 构建依赖
- `ament_cmake`
- `rosidl_default_generators`
- `geometry_msgs`

### 运行依赖
- `rosidl_default_runtime`
- `geometry_msgs`

## 接口定义

### 消息 (Messages)

| 消息类型 | 文件 | 说明 |
|----------|------|------|
| `ChatLLM` | `msg/ChatLLM.msg` | LLM 对话消息 |

### 服务 (Services)

| 服务类型 | 文件 | 说明 |
|----------|------|------|
| `ChatLLM` | `srv/ChatLLM.srv` | LLM 对话服务 |
| `MoveJoint` | `srv/MoveJoint.srv` | 关节角度移动服务 |
| `MoveCartesian` | `srv/MoveCartesian.srv` | 笛卡尔空间移动服务 |
| `RelativeMove` | `srv/RelativeMove.srv` | 相对移动服务 |
| `RecognizePickPlace` | `srv/RecognizePickPlace.srv` | 识别抓取放置服务 |

### 动作 (Actions)

| 动作类型 | 文件 | 说明 |
|----------|------|------|
| `ArmMoveTo` | `action/ArmMoveTo.action` | 机械臂移动到目标动作 |
| `MoveToPickPlace` | `action/MoveToPickPlace.action` | 抓取放置动作 |
| `Recognize` | `action/Recognize.action` | 识别动作 |
| `RecognizePickPlace` | `action/RecognizePickPlace.action` | 识别抓取放置动作 |

## 接口详情

### MoveJoint.srv
```yaml
float32[] angles  # 关节角度数组
---
bool success      # 执行结果
```

### MoveCartesian.srv
```yaml
# 多种笛卡尔运动格式支持
geometry_msgs/PoseStamped pose_stamped  # 位姿消息
float32[] pose_euler                     # 欧拉角
float32[] pose_quat                      # 四元数
string pose_link                        # 参考坐标系
---
bool success
```

### ChatLLM.srv
用于 LLM 函数调用服务的请求和响应。

## 关键文件

| 文件 | 说明 |
|------|------|
| `msg/ChatLLM.msg` | 对话消息定义 |
| `srv/ChatLLM.srv` | LLM 服务定义 |
| `srv/MoveJoint.srv` | 关节移动服务 |
| `srv/MoveCartesian.srv` | 笛卡尔移动服务 |
| `action/ArmMoveTo.action` | 机械臂动作定义 |
| `action/MoveToPickPlace.action` | 抓取放置动作定义 |
