# Sagittarius Toolbox 分析报告

> 分析日期: 2026-03-10

---

## 一、sagittarius_toolbox 目录概览

`sagittarius_toolbox` 目录包含 **3 个 ROS 2 包**：

| 包名 | 类型 | 功能 |
|------|------|------|
| `sagittarius_common_msgs` | ament_cmake | 自定义消息与服务定义 |
| `sagittarius_modules` | ament_python | Python 工具模块 |
| `sgr_ros_control` | ament_cmake | ROS 2 控制框架硬件接口 |

---

## 二、sagittarius_common_msgs - 自定义消息

### 消息 (msg)

| 消息名 | 功能 |
|--------|------|
| **ArmRadControl** | 目标位置数组 (float64[]) |
| **JointGroupCommand** | 控制关节组，支持位置模式 (name + cmd[]) |
| **JointSingleCommand** | 控制单个关节 (name + cmd) |
| **JointTrajectoryCommand** | 关节轨迹控制，支持 single/group 模式 |
| **SingleRadControl** | 单个舵机控制 (joint_name + rad) |

### 服务 (srv)

| 服务名 | 功能 |
|--------|------|
| **ArmInfo** | 获取机械臂信息：关节名、ID、限位、速度限制、夹爪限位、睡眠位置等 |
| **RobotInfo** | 获取指定关节组/关节的详细信息：模式、profile类型、关节名、ID、限位等 |
| **ServoRtInfo** | 获取舵机实时状态：速度、负载、电压、电流 |

---

## 三、sagittarius_modules - Python 工具模块

### sgr_common
- **功能**: 提供 Sagittarius 机械臂模型和关节定义
- **支持模型**: `sgr532`
- **关节列表**: joint1~joint6 + joint_gripper_left + joint_gripper_right

### sgr_launch
- **功能**: Launch 文件参数声明工具类
- **主要功能**:
  - `DeclareSagittariusArmDescriptionLaunchArgument` - 声明机械臂描述参数
  - `construct_sagittarius_arm_semantic_robot_description_command` - 构建 MoveIt SRDF 命令
  - `declare_sagittarius_arm_robot_description_launch_arguments` - 声明 launch 参数
  - `determine_use_sim_time_param` - 判断是否使用仿真时间

---

## 四、sgr_ros_control - ROS 控制

### SgrHardwareInterface (C++)

这是一个 **ros2_control 硬件接口插件**，实现 `hardware_interface::SystemInterface`：

| 方法 | 功能 |
|------|------|
| `on_init()` | 初始化：读取参数、创建 pub/sub、调用 ArmInfo 服务获取关节信息 |
| `export_state_interfaces()` | 导出位置、速度状态接口 |
| `export_command_interfaces()` | 导出位置命令接口 |
| `read()` | 从 joint_states 话题读取关节位置 |
| `write()` | 发布关节组命令和夹爪命令 |
| `joint_state_cb()` | 回调函数接收关节状态 |

### 关键特性
- 支持 6 自由度机械臂 + 夹爪
- 使用 MultiThreadedExecutor 处理异步消息
- 通过服务获取机械臂配置信息
- 只支持位置控制接口 (HW_IF_POSITION)

---

## 五、消息/服务定义汇总

### 消息类型

| 消息 | 字段 | 说明 |
|------|------|------|
| JointGroupCommand | name, cmd[] | 群控命令 |
| JointSingleCommand | name, cmd | 单关节命令 |
| JointTrajectoryCommand | trajectory | 轨迹命令 |
| ArmRadControl | joints[], positions[] | 弧度控制 |
| ArmInfo | joint_names[], joint_limits[] | 机械臂信息 |

### 服务类型

| 服务 | 请求 | 响应 |
|------|------|------|
| ServoRtInfo | empty | servo_id, velocity, voltage, current |
| ArmInfo | empty | joint_names, position_limits, velocity_limits |
| RobotInfo | joint_name | mode, profile, limits |

---

## 六、在项目中的使用

```
sdk_sagittarius_arm
    ↓ 依赖
sagittarius_common_msgs
    ↓ 提供消息/服务
    ↑
sagittarius_modules (Python)
sgr_ros_control (ros2_control hardware interface)
```

---

*该报告由 Daisy Agent 生成于 2026-03-10*
