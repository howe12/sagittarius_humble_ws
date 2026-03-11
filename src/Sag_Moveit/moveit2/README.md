# moveit2

MoveIt2 官方源码仓库 - ROS 2 运动规划框架

## 功能描述

MoveIt2 是 ROS 2 的官方运动规划框架，提供机器人 manipulation 功能的完整解决方案。该仓库包含 MoveIt2 的完整源码，是用于商业应用原型设计、算法基准测试的开源机器人操作平台。

### 主要子包

- **moveit_core**: 核心功能库
- **moveit_planners**: 运动规划器 (OMPL, STOMP, Pilz)
- **moveit_ros**: ROS 集成
  - moveit_ros_planning: 规划相关
  - moveit_ros_move_group: MoveGroup 节点
  - moveit_ros_visualization: Rviz 可视化
  - moveit_ros_perception: 感知集成
- **moveit_setup_assistant**: 配置助手
- **moveit_kinematics**: 逆运动学求解器
- **moveit_msgs**: ROS 消息定义
- **moveit_resources**: 示例机器人描述

## 依赖

- ROS 2 Humble/Iron/Rolling
- Eigen3
- OMPL
- Boost
- console_bridge
- tinyxml2

## 启动方式

该包为源码仓库，不直接启动。使用以下方式之一：

### 二进制安装
```bash
sudo apt install ros-humble-moveit
```

### 源码构建
```bash
cd ~/sagittarius_humble_ws
source /opt/ros/humble/setup.bash
vcs import src < src/Sag_Moveit/moveit2/moveit2/moveit2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --mixins release
```

## 关键文件

| 文件 | 说明 |
|------|------|
| `moveit2.repos` | VCS 仓库配置 |
| `README.md` | 官方文档 |
| `doc/MIGRATION_GUIDE.md` | 从 ROS 1 迁移指南 |

## 相关链接

- [官方文档](https://moveit.picknik.ai/)
- [GitHub 仓库](https://github.com/ros-planning/moveit2)
- [二进制安装](https://moveit.ros.org/install-moveit2/binary/)
- [源码构建](https://moveit.ros.org/install-moveit2/source/)
