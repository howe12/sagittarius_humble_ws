# sagittarius_gazebo

Sagittarius 机械臂 Gazebo 仿真功能包。

## 功能描述

该包提供 Sagittarius 机械臂在 Gazebo 仿真环境中的完整启动功能，包括：
- 自动生成 Gazebo 兼容的 URDF 文件（去除 ros2_control 配置）
- Robot State Publisher 节点启动
- Joint State Publisher GUI 用于手动控制关节
- Gazebo 仿真环境启动
- 机器人模型 Spawn 到仿真环境中
- 与 MoveIt 集成启动

## 依赖

### 构建依赖
- `ament_cmake`
- `ament_cmake_python`
- `gazebo_ros_pkgs`
- `xacro`

### 运行依赖
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

## 启动方式

### 标准启动
```bash
ros2 launch sagittarius_gazebo sagittarius_gazebo.launch.py
```

### 简化启动
```bash
ros2 launch sagittarius_gazebo sagittarius_gazebo_simple.launch.py
```

### 与 MoveIt 集成启动
```bash
ros2 launch sagittarius_gazebo sagittarius_moveit_gz.launch.py
```

### Shell 脚本启动
```bash
bash ~/sagittarius_humble_ws/src/sagittarius_gazebo/launch/start_gazebo.sh
```

## 关键文件

| 文件 | 说明 |
|------|------|
| `launch/sagittarius_gazebo.launch.py` | 主启动文件，包含完整仿真流程 |
| `launch/sagittarius_gazebo_simple.launch.py` | 简化版启动文件 |
| `launch/sagittarius_moveit_gz.launch.py` | MoveIt 集成启动文件 |
| `launch/start_gazebo.sh` | Shell 启动脚本 |
| `worlds/empty.world` | Gazebo 空世界文件 |
| `config/` | 配置文件目录 |
