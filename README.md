# Sagittarius Humble ROS2 机械臂项目

基于 ROS2 Humble 的 Sagittarius 机械臂控制项目，支持仿真和真实机械臂控制。

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## 项目结构

```
sagittarius_humble_ws/
├── src/
│   ├── Sag_Bringup/           # 机械臂启动/驱动包
│   │   ├── sdk_sagittarius_arm/   # 机械臂SDK
│   │   ├── sagittarius_descriptions/  # 机器人模型
│   │   └── ...
│   │
│   ├── Sag_Moveit/            # MoveIt运动规划配置
│   │   ├── sagittarius_humble_moveit/
│   │   └── sagittarius_moveit/
│   │
│   └── Sag_Vision/            # 视觉相关（Apriltag、YOLO、相机）
│
└── onekey.sh                 # 一键启动脚本
```

## 编译

```bash
cd sagittarius_humble_ws
source /opt/ros/humble/setup.bash
colcon build
```

## onekey.sh 功能说明

运行 `./onekey.sh` 可选择以下功能：

### 1. 启动机械臂仿真/真实控制
- **1** - 启动 RViz 仿真（虚拟机械臂）
- **2** - 启动 RViz 真实机械臂控制

### 2. 软链接设置
- **1** - 软连接机械臂设备（/dev/sagittarius）
- **2** - 软连接 USB 相机
- **3** - USB 相机标定
- **4** - 生成 Apriltag 码
- **5** - 眼在手上标定（Eye-in-Hand）
- **6** - 眼在手外标定（Eye-to-Hand）

### 3. 机械臂控制
- 启动 MoveIt 规划
- 关节位置控制
- 夹爪控制

### 4. 视觉功能
- Apriltag 检测
- YOLOv8 目标检测
- 相机标定

## 快速开始

### 1. 编译项目
```bash
cd sagittarius_humble_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 2. 运行仿真
```bash
# 方式一：使用 onekey.sh
./onekey.sh
# 选择 "1" 启动仿真

# 方式二：手动启动
source install/setup.bash
ros2 launch sagittarius_humble_moveit demo.launch.py
```

### 3. 运行真实机械臂
```bash
# 确保机械臂已连接
ls -la /dev/sagittarius*

# 启动控制
./onekey.sh
# 选择 "2" 启动真实机械臂控制
```

## 机械臂型号

- **SGR532** - 6轴协作机械臂

## 关节名称
- joint1 ~ joint6（6个关节）
- joint_gripper_left（夹爪）

## 注意事项

1. 首次使用需确保机械臂 USB 设备已正确连接
2. 运行前需要 `source install/setup.bash`
3. 真实机械臂需要设置 `/dev/sagittarius` 软链接

## 相关文档

- 机械臂SDK: `src/Sag_Bringup/sdk_sagittarius_arm/`
- MoveIt配置: `src/Sag_Moveit/sagittarius_humble_moveit/`
- URDF模型: `src/Sag_Bringup/sagittarius_descriptions/urdf/`

## 联系支持

- 技术支持: litian.zhuang@nxrobo.com
- QQ群: 6646169, 8346256

---
*更新于 2026-03-10*
