# Sagittarius Toolbox 分析报告

> 分析日期: 2026-03-10

---

## 一、sagittarius_toolbox 目录概览

`/home/leo/Music/sagittarius_humble_ws/src/Sag_Bringup/sagittarius_toolbox` 包含机械臂公共工具包。

---

## 二、各包详细分析

### 1. sagittarius_common_msgs - 自定义消息

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_cmake |
| **功能** | 定义 Sagittarius 机械臂的 ROS2 消息/服务 |

**自定义消息 (`msg/`):**
| 消息名 | 类型 | 用途 |
|--------|------|------|
| `JointGroupCommand.msg` | 消息 | 6轴同时控制命令 |
| `JointSingleCommand.msg` | 消息 | 单关节控制命令 |
| `JointTrajectoryCommand.msg` | 消息 | 轨迹跟踪命令 |
| `ArmRadControl.msg` | 消息 | 弧度控制命令 |
| `ArmInfo.msg` | 消息 | 机械臂信息 |

**自定义服务 (`srv/`):**
| 服务名 | 类型 | 用途 |
|--------|------|------|
| `ServoRtInfo.srv` | 服务 | 获取舵机实时信息（速度、电压、电流、温度） |
| `ArmInfo.srv` | 服务 | 获取机械臂信息 |

**关键消息定义：**

```yaml
# JointGroupCommand.msg
float64[] positions    # 关节位置（弧度）
float64[] velocities   # 关节速度
float64[] accelerations  # 关节加速度
```

---

### 2. sagittarius_modules - Python 工具模块

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_python |
| **功能** | 提供 Python 公共工具函数和类 |

**主要模块：**
| 模块名 | 功能 |
|--------|------|
| `sgr_common.py` | 通用工具函数（获取机械臂型号等） |
| `sgr_kinematics.py` | 运动学计算 |
| `sgr_robot.py` | 机器人操作封装 |

---

### 3. sgr_ros_control - ROS 控制工具

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_cmake |
| **功能** | ROS 控制相关工具 |

**功能：**
- 控制器相关工具
- 参数配置工具

---

## 三、消息/服务定义汇总

### 消息类型

| 消息 | 字段 | 说明 |
|------|------|------|
| JointGroupCommand | positions[], velocities[], accelerations[] | 群控命令 |
| JointSingleCommand | joint_name, position, velocity | 单关节命令 |
| JointTrajectoryCommand | trajectory | 轨迹命令 |
| ArmRadControl | joints[], positions[] | 弧度控制 |
| ArmInfo | joint_names[], joint_limits[] | 机械臂信息 |

### 服务类型

| 服务 | 请求 | 响应 |
|------|------|------|
| ServoRtInfo | empty | servo_id, velocity, voltage, current, temperature |
| ArmInfo | empty | joint_names, position_limits, velocity_limits |

---

## 四、在项目中的使用

```
sdk_sagittarius_arm
    ↓ 依赖
sagittarius_common_msgs
    ↓ 提供消息/服务
    ↑
sagittarius_modules (Python)
sgr_ros_control
```

---

*该报告由 Daisy Agent 生成于 2026-03-10*
