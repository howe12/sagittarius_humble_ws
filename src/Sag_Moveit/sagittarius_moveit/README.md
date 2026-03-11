# sagittarius_moveit

Sagittarius 机械臂 MoveIt2 配置包 (ROS 2 版本)

## 功能描述

为 Sagittarius sgr532 机械臂提供的 MoveIt2 配置包，支持运动规划、逆运动学求解和轨迹控制。

主要功能：
- 机械臂运动规划
- 逆运动学 (IK) 求解
- 碰撞检测与场景管理
- 轨迹执行与控制
- 支持 Gazebo 仿真

## 依赖

### 核心依赖
- `moveit_ros_move_group`
- `moveit_kinematics`
- `moveit_planners`
- `moveit_planners_ompl` - OMPL 规划器
- `moveit_simple_controller_manager`
- `sagittarius_descriptions` - 机器人描述
- `sagittarius_modules`

### 可视化
- `rviz2`
- `moveit_ros_visualization`
- `joint_state_publisher_gui`

### 其他
- `xacro`
- `tf2_ros`
- `warehouse_ros_mongo`

## 启动方式

### 1. RViz 演示模式
```bash
ros2 launch sagittarius_moveit demo.launch.py
```

### 2. 真实机械臂模式
```bash
ros2 launch sagittarius_moveit demo_true.launch.py
```

### 3. 完整演示 (含所有功能)
```bash
ros2 launch sagittarius_moveit full_demo.launch.py
```

### 4. MoveIt + RViz
```bash
ros2 launch sagittarius_moveit moveit_rviz.launch.py
```

### 5. MoveGroup 节点
```bash
ros2 launch sagittarius_moveit move_group.launch.py
```

### 6. 机器人状态发布
```bash
ros2 launch sagittarius_moveit rsp.launch.py
```

### 7. Setup Assistant
```bash
ros2 launch sagittarius_moveit setup_assistant.launch.py
```

## 关键文件

### Launch 文件
| 文件 | 说明 |
|------|------|
| `demo.launch.py` | 演示模式 |
| `demo_true.launch.py` | 真实机械臂模式 |
| `full_demo.launch.py` | 完整演示 |
| `moveit_rviz.launch.py` | MoveIt + RViz |
| `move_group.launch.py` | MoveGroup 节点 |

### 配置文件
| 文件 | 说明 |
|------|------|
| `config/ompl_planning_true.yaml` | OMPL 规划配置 |
| `config/kinematics.yaml` | 运动学参数 |
| `config/kinematics_actual.yaml` | 实际运动学 |
| `config/joint_limits.yaml` | 关节限位 |
| `config/moveit_controllers.yaml` | 控制器配置 |
| `config/ros2_controllers.yaml` | ROS2 控制器 |
| `config/sensors_3d.yaml` | 3D 传感器配置 |
| `config/modes.yaml` | 运行模式配置 |

### 控制器配置
| 文件 | 说明 |
|------|------|
| `config/controllers/sgr532_controllers.yaml` | 控制器定义 |
| `config/controllers/sgr532_ros_controllers.yaml` | ROS2 控制器 |

### SRDF 配置
- `config/srdf/sgr532.srdf.xacro`
- `config/sgr532.srdf.xacro`

## 与 sagittarius_humble_moveit 的区别

- `sagittarius_moveit`: 更通用的版本，可能用于 ROS 1 或较早的 ROS 2 版本
- `sagittarius_humble_moveit`: 专用于 ROS 2 Humble 版本的配置

具体选择取决于 ROS 2 发行版和机械臂固件版本。
