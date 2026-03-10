# Sagittarius Arm 项目架构分析报告

> 分析日期: 2026-03-10

---

## 1. 项目整体结构

```
/home/leo/Music/sagittarius_humble_ws/
├── src/
│   ├── Sag_Audio/          # 音频输入
│   ├── Sag_Bringup/        # 启动包（核心）
│   │   ├── sdk_sagittarius_arm/     # 机械臂驱动 ⭐
│   │   ├── sagittarius_descriptions/ # URDF/XACRO 描述
│   │   └── sagittarius_toolbox/      # 公共工具箱
│   │       ├── sagittarius_common_msgs/  # 自定义消息
│   │       ├── sagittarius_modules/       # Python 工具
│   │       └── sgr_ros_control/           # ROS 控制
│   ├── Sag_Demo/           # 演示应用
│   │   ├── cobot_draw/
│   │   ├── sagittarius_py_demo/
│   │   └── ...
│   ├── Sag_Moveit/         # MoveIt 配置 ⭐
│   │   ├── sagittarius_humble_moveit/  # MoveIt 配置
│   │   ├── sagittarius_moveit/          # 旧版配置
│   │   └── moveit2/                     # MoveIt2 源码
│   └── Sag_Vision/         # 视觉模块
│       ├── apriltag/
│       ├── ros2_yolov8/
│       ├── realsense2_description/
│       └── usb_cam/
```

---

## 2. 核心包功能分析

### 2.1 `sdk_sagittarius_arm` - 机械臂驱动

| 属性 | 值 |
|------|-----|
| **类型** | ament_cmake (C++) |
| **节点** | `sdk_sagittarius_arm_node` |
| **主要功能** | 串口通信控制机械臂 |

**依赖:**
- `rclcpp`, `sensor_msgs`, `urdf`, `yaml-cpp`
- `moveit_msgs`, `sagittarius_common_msgs`, `std_msgs`
- `Boost`

**关键源文件:**
- `sdk_sagittarius_arm_real.cpp` - 实际驱动实现
- `sdk_sagittarius_arm_common.cpp` - 通用通信层
- `sdk_sagittarius_arm_common_serial.cpp` - 串口通信

---

### 2.2 `sagittarius_descriptions` - 机器人模型

| 属性 | 内容 |
|------|------|
| **URDF** | `sgr532.urdf.xacro` |
| **控制文件** | `control.urdf.xacro` |
| **Mesh** | `meshes/` 目录 |

---

### 2.3 `sagittarius_common_msgs` - 自定义消息

| 消息类型 | 用途 |
|---------|------|
| `JointGroupCommand` | 6轴同时控制 |
| `JointSingleCommand` | 单关节控制 |
| `JointTrajectoryCommand` | 轨迹跟踪 |
| `ArmRadControl` | 弧度控制 |
| `ServoRtInfo.srv` | 舵机实时信息服务 |

---

## 3. ros2_control 配置

### 3.1 控制器配置 (`ros2_controllers.yaml`)

```yaml
controller_manager:
  update_rate: 100Hz
  
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints: [joint1~joint6]
  command_interfaces: [position]
  state_interfaces: [position, velocity]

gripper_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints: [joint_gripper_left]
  command_interfaces: [position]
  state_interfaces: [position, velocity]
```

### 3.2 ros2_control xacro (`sgr532.ros2_control.xacro`)

- **默认硬件**: `mock_components/GenericSystem` (仿真)
- **关节**: joint1~joint6 + joint_gripper_left
- **接口**: position command + position/velocity state

---

## 4. MoveIt 配置

### 4.1 关键配置文件

| 文件 | 用途 |
|------|------|
| `sgr532.srdf` | 语义机器人描述 |
| `sgr532.urdf.xacro` | 运动学模型 |
| `kinematics_actual.yaml` | 运动学求解器配置 |
| `joint_limits.yaml` | 关节限位 |
| `ompl_planning.yaml` | OMPL 规划管道 |
| `moveit_controllers.yaml` | MoveIt 控制器配置 |

### 4.2 运动学配置

```yaml
# kinematics.yaml
robot_name: sgr532
groups:
  arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

---

## 5. 消息传递机制

### 5.1 硬件 ↔ ROS2

```
串口 (RS485)  →  sdk_sagittarius_arm  →  sensor_msgs::JointState
                                    ↑                         ↓
                           sagittarius_common_msgs  ←  控制命令
                                    ↑                         ↓
                           joint_trajectory_controller
```

### 5.2 关键 Topic

| Topic | 类型 | 方向 |
|------|------|------|
| `joint_states` | `sensor_msgs/JointState` | 驱动 → ROS |
| `commands/joint_group` | `JointGroupCommand` | 群控 |
| `commands/joint_single` | `JointSingleCommand` | 单关节 |
| `commands/joint_trajectory` | `JointTrajectoryCommand` | 轨迹 |
| `control_torque` | `std_msgs/String` | 力矩开关 |

### 5.3 服务

| 服务 | 类型 | 用途 |
|------|------|------|
| `get_servo_info` | `ServoRtInfo.srv` | 获取舵机实时信息 |
| `get_robot_info` | `ArmInfo.srv` | 获取机器人信息 |

---

## 6. 关键 Launch 文件

| Launch 文件 | 用途 |
|------------|------|
| `sagittarius_moveit.launch.py` | **主启动文件** - MoveIt + SDK 集成 |
| `sagittarius_arm_sdk.launch.py` | 单独启动 SDK 驱动 |
| `sagittarius_description_true.launch.py` | 真实机器人描述 |

### 主启动流程 (`sagittarius_moveit.launch.py`)

```
1. robot_description (xacro)
   ↓
2. robot_description_semantic (SRDF)
   ↓
3. sdk_sagittarius_arm_node (驱动)
   ↓
4. move_group (MoveIt)
   ↓
5. controller_manager (ros2_control)
   ↓
6. arm_controller + gripper_controller
```

---

## 7. 依赖关系图

```
sagittarius_humble_moveit
         ↓
sagittarius_descriptions ← sagittarius_common_msgs
         ↓                         ↑
sdk_sagittarius_arm ←──────┘
         ↓
   串口硬件 (SGR532 机械臂)
```

---

## 8. 架构总结

| 层面 | 实现 |
|------|------|
| **硬件抽象** | ros2_control + mock_components |
| **运动规划** | MoveIt2 + OMPL |
| **驱动通信** | 串口 (RS485) + 自定义协议 |
| **消息传递** | 自定义 msgs + 标准 sensor_msgs |
| **配置管理** | YAML + XACRO 参数化 |

---

*该报告由 Richard Agent 生成于 2026-03-10*
