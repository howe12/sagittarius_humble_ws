# cobot_draw

协作机器人绘图功能包 - 使用 MoveIt 实现轨迹规划与绘图控制

## 功能描述

该功能包提供基于 MoveIt 的协作机器人绘图功能，支持多种绘图轨迹生成与执行：

- **draw_gcode**: 解析 GCode 文件并生成机器人绘图轨迹
- **draw_five_stars**: 生成五角星绘图轨迹
- **draw_circle**: 生成圆形绘图轨迹  
- **draw_square**: 生成方形绘图轨迹
- **get_current_pose**: 获取机器人当前末端姿态

## 依赖

### 系统依赖
- ROS 2 (Humble)
- MoveIt 2

### ROS 依赖
- `rclcpp`
- `moveit_core`
- `moveit_ros_planning_interface`
- `tf2_geometry_msgs`
- `geometry_msgs`
- `moveit_msgs`

### 构建工具
- `ament_cmake`

## 构建与安装

```bash
cd ~/sagittarius_humble_ws
source install/setup.bash
colcon build --packages-select cobot_draw
```

## 启动方式

### 绘制五角星
```bash
ros2 launch cobot_draw draw_five_stars_true.launch.py
```

### 绘制圆形
```bash
ros2 launch cobot_draw draw_circle_true.launch.py
```

### 绘制方形
```bash
ros2 launch cobot_draw draw_square_true.launch.py
```

### 绘制 GCode
```bash
ros2 launch cobot_draw draw_gcode.launch.py
```

### 获取当前姿态
```bash
ros2 run cobot_draw get_current_pose
```

## 关键文件

| 文件路径 | 说明 |
|---------|------|
| `src/draw_gcode.cpp` | GCode 解析与轨迹生成 |
| `src/draw_five_stars.cpp` | 五角星轨迹生成 |
| `src/draw_circle.cpp` | 圆形轨迹生成 |
| `src/draw_square.cpp` | 方形轨迹生成 |
| `src/get_current_pose.cpp` | 获取当前末端姿态 |
| `launch/draw_five_stars_true.launch.py` | 五角星绘制启动文件 |
| `launch/draw_circle_true.launch.py` | 圆形绘制启动文件 |
| `launch/draw_square_true.launch.py` | 方形绘制启动文件 |
| `config/` | 配置文件目录 |

## 配置

配置文件位于 `config/` 目录，包含机器人 URDF、SRDF 等配置。
