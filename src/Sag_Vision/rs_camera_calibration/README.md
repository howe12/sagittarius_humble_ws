# rs_camera_calibration

RealSense 相机标定包

## 功能描述

用于 Intel RealSense 深度相机的标定工具，支持：
- 相机内参标定 (相机矩阵、畸变系数)
- RealSense D400 系列相机标定
- 与 MoveIt2 集成的手眼标定

## 依赖

### 核心依赖
- `moveit_ros_planning_interface` - MoveIt 规划接口
- `moveit_visual_tools` - MoveIt 可视化工具

### 系统依赖
- OpenCV
- Camera Calibrator (ROS 标定工具)

## 启动方式

### 1. 启动 RealSense 相机
```bash
ros2 launch realsense2_camera rs_launch.py
```

### 2. 启动相机标定
```bash
ros2 launch rs_camera_calibration rs_camera_calibration.launch.py
```

### 3. 使用 Sagittarius 机械臂标定
```bash
ros2 launch rs_camera_calibration sag_rs_camera_calibration.launch.py
```

## 关键文件

### Launch 文件
| 文件 | 说明 |
|------|------|
| `launch/rs_camera_calibration.launch.py` | 通用标定启动 |
| `launch/sag_rs_camera_calibration.launch.py` | Sagittarius 专用标定 |

### 配置目录
| 文件 | 说明 |
|------|------|
| `config/` | 标定配置文件 |

### 脚本目录
| 文件 | 说明 |
|------|------|
| `scripts/` | 标定脚本 |

### 标定板
| 文件 | 说明 |
|------|------|
| `charuco_board_5x5_100.png` | Charuco 标定板图像 |

## 标定流程

### 步骤 1: 启动 RealSense 相机
```bash
ros2 launch realsense2_camera rs_launch.py
```

### 步骤 2: 运行标定
```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.108 \
    image:=/camera/color/image_raw
```

### 步骤 3: 保存标定结果
标定完成后，结果将保存到 `~/.ros/camera_info/` 目录

## Charuco 标定板

提供的 Charuco 标定板参数：
- 棋盘格大小: 5×5
- 每个方格: 100mm
- 可打印使用

## 手眼标定

对于机械臂+相机的系统，可进行手眼标定：

```bash
ros2 launch rs_camera_calibration sag_rs_camera_calibration.launch.py
```

此功能用于确定相机相对于机械臂末端或基座的变换关系。
