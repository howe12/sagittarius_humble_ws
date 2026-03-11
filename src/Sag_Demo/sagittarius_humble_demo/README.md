# sagittarius_humble_demo

Sagittarius 机器人控制演示功能包

## 功能描述

该功能包是 Sagittarius 协作机器人在 ROS 2 Humble 平台上的演示框架，包含：

### sagittarius_puppet_control (木偶控制)

基于 ROS 2 的机器人木偶控制功能，实现示教与复现功能：

- **puppet_control_single_node**: 单节点木偶控制
- **ros2bag_record_control**: ROS2 bag 录制控制

## 依赖

### 系统依赖
- ROS 2 (Humble)
- Python 3.8+

### ROS 依赖
- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `sagittarius_common_msgs` (Sagittarius 通用消息)
- `rosbag2_transport`

### 构建工具
- `ament_cmake`

## 构建与安装

```bash
cd ~/sagittarius_humble_ws
source install/setup.bash
colcon build --packages-select sagittarius_puppet_control
```

## 启动方式

### 木偶控制节点
```bash
ros2 run sagittarius_puppet_control puppet_control_single_node
```

### Ros2bag 录制控制
```bash
ros2 run sagittarius_puppet_control ros2bag_record_control
```

## 关键文件

| 文件路径 | 说明 |
|---------|------|
| `sagittarius_puppet_control/src/puppet_control_single_node.cpp` | 木偶控制主节点 |
| `sagittarius_puppet_control/src/ros2bag_record_control.cpp` | Ros2bag 录制控制 |
| `sagittarius_puppet_control/launch/` | 启动文件目录 |
| `sagittarius_puppet_control/config/` | 配置文件目录 |
| `sagittarius_puppet_control/rviz/` | RViz 配置目录 |

## 配置文件

配置文件位于 `sagittarius_puppet_control/config/` 目录，包含机器人配置、控制器参数等。

## RViz 配置

RViz 配置文件位于 `sagittarius_puppet_control/rviz/` 目录。
