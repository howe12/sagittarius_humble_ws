# sdk_sagittarius_arm

ROS2 Sagittarius 机械臂 SDK 包，提供机械臂的实时控制、串口通信和 MoveIt 集成。

## 功能描述

该包提供 Sagittarius 机械臂的完整 SDK：

- **机械臂控制**：提供 C++ 节点实现机械臂运动控制
- **串口通信**：通过串口与机械臂硬件通信
- **MoveIt 集成**：支持 MoveIt 运动规划
- **轨迹执行**：支持轨迹消息的接收和执行
- **Gazebo 仿真**：支持仿真环境下的控制

## 依赖

- `ament_cmake` - CMake 构建系统
- `rclcpp` - ROS2 C++ 客户端库
- `rclpy` - ROS2 Python 客户端库
- `sensor_msgs` - 传感器消息
- `std_msgs` - 标准消息
- `moveit_msgs` - MoveIt 消息
- `urdf` - URDF 工具库
- `yaml-cpp` - YAML C++ 解析库
- `Boost` - Boost C++ 库
- `sagittarius_common_msgs` - Sagittarius 公共消息

## 启动方式

### 方式 1：启动 SDK 节点

```bash
# 启动机械臂 SDK
ros2 run sdk_sagittarius_arm sdk_sagittarius_arm_node
```

### 方式 2：使用 Launch 文件

```bash
# 基础 SDK 启动
ros2 launch sdk_sagittarius_arm sagittarius_arm_sdk.launch.py

# 真实机器人描述启动
ros2 launch sdk_sagittarius_arm sagittarius_description_true.launch.py

# MoveIt 启动
ros2 launch sdk_sagittarius_arm sagittarius_moveit.launch.py
```

## 关键文件

```
sdk_sagittarius_arm/
├── src/
│   ├── sdk_sagittarius_arm.cpp              # 主入口
│   ├── sdk_sagittarius_arm_common.cpp       # 通用实现
│   ├── sdk_sagittarius_arm_common_serial.cpp # 串口通信
│   └── sdk_sagittarius_arm_real.cpp         # 真实机器人控制
├── include/sdk_sagittarius_arm/
│   ├── sdk_sagittarius_arm_common.h         # 通用头文件
│   ├── sdk_sagittarius_arm_common_serial.h  # 串口头文件
│   ├── sdk_sagittarius_arm_constants.h      # 常量定义
│   └── sdk_sagittarius_arm_real.h           # 真实机器人头文件
├── launch/
│   ├── sagittarius_arm_sdk.launch.py        # SDK 启动
│   ├── sagittarius_description_true.launch.py  # 真实机器人描述
│   └── sagittarius_moveit.launch.py         # MoveIt 启动
├── cfg/
│   └── sgr532.yaml                          # 机械臂配置参数
├── rviz/
│   └── moveit.rviz                          # MoveIt RViz 配置
├── CMakeLists.txt
└── package.xml
```

## 话题和服务

### 订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/sagittarius_arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | 轨迹执行 |
| `/sagittarius_arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | 跟踪轨迹 |

### 发布的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 关节状态 |
| `/sagittarius_arm_controller/state` | control_msgs/ControllerState | 控制器状态 |

## 配置参数 (sgr532.yaml)

配置文件包含机械臂的关节限制、速度、加速度等参数：

- 关节位置限制
- 关节速度限制
- 关节加速度限制
- 串口配置
- 控制参数

## 使用示例

### 轨迹执行

```bash
# 发布轨迹命令
ros2 topic pub /sagittarius_arm_controller/joint_trajectory trajectory_msgs/JointTrajectory "..."
```

### MoveIt 控制

```bash
# 启动 MoveIt 后，可使用 MoveIt! RViz 插件进行交互式控制
ros2 launch sdk_sagittarius_arm sagittarius_moveit.launch.py
```

---
Maintainer: spark (litian.zhuang@nxrobo.com)
License: TODO
