# Sagittarius Arm SDK 代码分析报告

> 分析日期: 2026-03-10

---

## 一、项目概述

`/home/leo/Music/sagittarius_humble_ws` 项目是一个基于 ROS2 Humble 的机械臂控制系统，支持 Sagittarius (SGR532) 机械臂。项目使用串口通信协议与机械臂硬件进行交互，并通过 ros2_control 框架与 MoveIt! 集成实现运动规划。

---

## 二、核心代码分析 (sdk_sagittarius_arm/src/)

### 2.1 sdk_sagittarius_arm.cpp - 主入口

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool success = true;
  auto node = std::make_shared<sdk_sagittarius_arm::SagittariusArmReal>(success);
  if (success) {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  }
  // ...
}
```

**功能**：初始化 ROS2 节点，创建 `SagittariusArmReal` 节点并运行多线程执行器。

---

### 2.2 sdk_sagittarius_arm_common.cpp - 通用基类

**类**: `CSDarmCommon`

**核心功能**:
1. **协议解析**: 实现基于 0x55AA 帧头的通信协议
2. **校验和计算**: `CheckSum()` 函数用于验证数据完整性
3. **数据帧处理**: `LoopOnce()` 接收并解析串口数据

**通信协议格式**:
```
[0x55][0xAA][数据长度][类型][命令][数据...][校验和][0x7D]
```

**关键命令码** (定义于 sdk_sagittarius_arm_constants.h):
- `0x05`: CMD_CONTROL_ALL_DEGREE - 多关节角度控制
- `0x0B`: CMD_CONTROL_ALL_DEGREE_CB - 插补控制命令
- `0x0C`: CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME - 带时间差的控制
- `0x08`: CMD_CONTROL_LOCK_OR_FREE - 舵机锁/释放
- `0x10`: CMD_GET_SERVO_RT_INFO - 获取舵机实时信息

---

### 2.3 sdk_sagittarius_arm_common_serial.cpp - 串口通信实现

**类**: `CSDarmCommonSerial` (继承 CSDarmCommon)

**核心功能**:

| 函数 | 功能 |
|------|------|
| `InitDevice()` | 串口初始化，配置波特率(1500000/1000000/460800/230400/115200等)、8N1(8位数据、无校验、1位停止位) |
| `SendSerialData2Arm()` | 发送数据到串口(带互斥锁保护) |
| `SendArmAllServer()` | 直接发送6个关节角度目标 |
| `SendArmAllServerCB()` | 插补方式控制(仅发送变化部分) |
| `SendArmAllServerTime()` | 带时间差的轨迹控制 |
| `SetArmVel()` | 设置舵机插补速度 |
| `SetArmAcc()` | 设置舵机加速度 |
| `GetDataGram()` | 使用 select() 多路复用读取串口数据 |

**数据转换**:
- 弧度 → 度 × 10: `degree = 1800/PI * v1`
- 夹爪距离 → 角度: `result = -(3462 * half_dist) * 10`

---

### 2.4 sdk_sagittarius_arm_real.cpp - ROS2 节点实现

**类**: `SagittariusArmReal` (继承 rclcpp::Node)

**初始化流程**:
```cpp
SagittariusArmReal::SagittariusArmReal(bool &success) {
    sgr_init_parameters();      // 1. 参数初始化
    sgr_init_driver();         // 2. 驱动初始化
    sgr_init_topics();         // 3. 话题订阅/发布
    sgr_wait_for_joint_states(); // 4. 等待关节状态
    sgr_init_setup();          // 5. 舵机参数配置
    get_param_from_yaml_urdf(); // 6. 加载URDF参数
    sgr_init_services();      // 7. 服务创建
}
```

**订阅的话题**:
| 话题 | 消息类型 | 功能 |
|------|----------|------|
| `commands/joint_group` | JointGroupCommand | 6轴同时控制 |
| `commands/joint_single` | JointSingleCommand | 单轴/夹爪控制 |
| `commands/joint_trajectory` | JointTrajectoryCommand | 轨迹控制(未实现) |
| `joint_states` | JointState | RViz仿真模式输入 |
| `control_torque` | String | 扭矩开关控制 |

**发布的话题**:
| 话题 | 消息类型 | 功能 |
|------|----------|------|
| `joint_states` | sensor_msgs/JointState | 发布当前关节角度 |

**提供的服务**:
| 服务 | 类型 | 功能 |
|------|------|------|
| `get_robot_info` | ArmInfo | 获取机器人信息(关节限位、名称等) |
| `get_servo_info` | ServoRtInfo | 获取舵机实时信息(速度、电压、电流) |

**关键回调**:
- `JointStatesCb()`: 从 `/joint_states` 读取目标位置，通过 `SendArmAllServerCB()` 发送到机械臂
- `sgr_sub_command_group()`: 处理 `JointGroupCommand` 消息

---

## 三、消息传递机制分析

### 3.1 ros2_control 集成方式

本项目**未使用**标准的 ros2_control hardware_interface，而是采用**直接话题通信**方式：

```
MoveIt! 
    ↓ /sgr532/arm_controller/follow_joint_trajectory (Action)
joint_trajectory_controller
    ↓ /sgr532/joint_states (Topic)
sdk_sagittarius_arm_node
    ↓ Serial
SGR532 Mechanical Arm
```

### 3.2 数据流分析

1. **上位机 → 机械臂**:
   - MoveIt! 规划 → `/sgr532/arm_controller/follow_joint_trajectory` action
   - joint_trajectory_controller 解析轨迹
   - 通过 `/sgr532/joint_states` topic 发送目标位置
   - `sdk_sagittarius_arm` 节点订阅并转发到串口

2. **机械臂 → 上位机**:
   - 机械臂通过串口发送实时关节状态 (0x06 命令)
   - `LoopOnce()` 解析数据帧
   - `PublishJointStates()` 发布到 `/joint_states`

---

## 四、启动流程分析

### 4.1 sagittarius_arm_sdk.launch.py

```python
sdk_sgr_node = Node(
    package='sdk_sagittarius_arm',
    executable='sdk_sagittarius_arm_node',
    parameters=[{
        'robot_model': 'sgr532',
        'robot_name': 'sgr532',
        'joint_configs': '.../sgr532.yaml',
        'serial_port': '/dev/sagittarius_0',
        'just_rviz_control': 'false',
        'exit_free_torque': 'false',
    }]
)
```

**启动参数**:
- `robot_model`: 机械臂型号 (sgr532)
- `serial_port`: 串口设备路径
- `just_rviz_control`: 仅RViz仿真模式
- `exit_free_torque`: 退出时是否释放扭矩

### 4.2 sagittarius_moveit.launch.py - 完整启动链

```
launch 文件启动顺序:
├── robot_state_publisher (发布TF)
├── sdk_sagittarius_arm_node (机械臂驱动)
├── controller_manager (ros2_control)
│   ├── arm_controller (joint_trajectory_controller)
│   └── gripper_controller
├── move_group (MoveIt!规划)
└── rviz2 (可视化)
```

---

## 五、机械臂控制数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                        数据流向图                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────┐      /arm_controller/follow_joint_trajectory   │
│   │   MoveIt!   │ ───────────────────────────────────────────────► │
│   │  (规划器)   │                                                   │
│   └──────────────┘                                                   │
│           │                                                         │
│           ▼                                                         │
│   ┌──────────────────────┐      /joint_states                      │
│   │ joint_trajectory_    │ ─────────────────────────────────────► │
│   │ controller           │     (sensor_msgs/JointState)            │
│   └──────────────────────┘                                         │
│           │                                                         │
│           ▼                                                         │
│   ┌──────────────────────┐      /joint_states                      │
│   │ sdk_sagittarius_arm  │ ◄────────────────────────────────────  │
│   │ 节点                 │     (订阅控制器输出)                     │
│   └──────────────────────┘                                         │
│           │                                                         │
│           ▼                                                         │
│   ┌──────────────────────┐      Serial (USB)                       │
│   │ CSDarmCommonSerial   │ ─────────────────────────────────────► │
│   │ 串口通信层           │     0x55 0xAA 0x0B ...                   │
│   └──────────────────────┘                                         │
│           │                                                         │
│           ▼                                                         │
│   ┌──────────────────────┐                                          │
│   │    SGR532 机械臂     │ ◄───────────────────────────────────   │
│   │    (6轴 + 夹爪)      │     串口返回关节状态                      │
│   └──────────────────────┘                                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 六、关键配置文件

### sgr532.yaml (关节配置)
```yaml
joint_num: 6
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint_gripper_left]
sleep: [0.0, 1.4, -1.47, 0.0, -0.4851, 0.0]
```

---

## 七、总结

| 方面 | 实现方式 |
|------|----------|
| 通信协议 | 自定义二进制协议 (0x55AA帧头) |
| 硬件接口 | 串口直接通信 (非 ros2_control SystemInterface) |
| 控制器 | joint_trajectory_controller (通过话题交互) |
| 运动规划 | MoveIt! + ompl |
| 夹爪控制 | 单独的话题/服务接口 |

**注意**: 该架构采用了一种混合模式：虽然配置了 ros2_controllers.yaml，但实际硬件控制是通过**话题订阅**方式实现的，而非标准的 ros2_control hardware interface。

---

*该报告由 Ross Agent 生成于 2026-03-10*
