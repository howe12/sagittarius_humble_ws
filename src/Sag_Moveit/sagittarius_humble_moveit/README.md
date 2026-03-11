# sagittarius_humble_moveit

Sagittarius 机械臂 MoveIt2 配置包 (ROS 2 Humble 版本)

## 功能描述

自动生成的 MoveIt2 配置包，为 Sagittarius sgr532 机械臂提供完整的运动规划功能。

主要功能：
- 机械臂运动规划与控制
- 逆运动学求解
- 碰撞检测
- 轨迹执行
- 与 Gazebo 仿真集成

## 依赖

### 核心依赖
- `moveit_ros_move_group` - MoveGroup 节点
- `moveit_kinematics` - 逆运动学
- `moveit_planners` - 规划器 (OMPL)
- `moveit_simple_controller_manager` - 控制器管理
- `sagittarius_descriptions` - 机器人描述

### 可视化与工具
- `rviz2` - 3D 可视化
- `moveit_ros_visualization` - MoveIt 可视化
- `joint_state_publisher` / `joint_state_publisher_gui` - 关节状态发布
- `robot_state_publisher` - 机器人状态发布

### 其他
- `xacro` - Xacro 预处理
- `tf2_ros` - TF2 变换
- `warehouse_ros_mongo` - 规划场景数据库

## 启动方式

### 1. RViz 演示模式 (无真实机械臂)
```bash
ros2 launch sagittarius_humble_moveit demo.launch.py
```

### 2. 带有 RViz 的 MoveIt
```bash
ros2 launch sagittarius_humble_moveit moveit_rviz.launch.py
```

### 3. MoveGroup 节点 (程序控制)
```bash
ros2 launch sagittarius_humble_moveit move_group.launch.py
```

### 4. 机器人状态发布
```bash
ros2 launch sagittarius_humble_moveit rsp.launch.py
```

### 5. Gazebo 仿真模式
```bash
ros2 launch sagittarius_humble_moveit moveit_gazebo_v2.launch.py
```

### 6. Setup Assistant 配置
```bash
ros2 launch sagittarius_humble_moveit setup_assistant.launch.py
```

## 关键文件

### Launch 文件
| 文件 | 说明 |
|------|------|
| `demo.launch.py` | RViz 演示模式 |
| `moveit_rviz.launch.py` | MoveIt + RViz |
| `move_group.launch.py` | MoveGroup 节点 |
| `rsp.launch.py` | 机器人状态发布 |
| `moveit_gazebo_v2.launch.py` | Gazebo 仿真 |
| `warehouse_db.launch.py` | 数据库启动 |

### 配置文件
| 文件 | 说明 |
|------|------|
| `config/ompl_planning.yaml` | OMPL 规划器配置 |
| `config/kinematics.yaml` | 运动学参数 |
| `config/joint_limits.yaml` | 关节限位 |
| `config/moveit_controllers.yaml` | 控制器配置 |
| `config/ros2_controllers.yaml` | ROS2 控制器 |
| `config/sgr532.urdf.xacro` | 机器人 URDF |

## 规划器配置

默认使用 OMPL 规划器，支持的规划算法：
- RRTConnect
- RRT*
- PRM
- PRM*
- BKPIECE1
- LBKPIECE1
- KPIECE1
- SBL
- STOMP (需单独配置)
- CHOMP (需单独配置)
