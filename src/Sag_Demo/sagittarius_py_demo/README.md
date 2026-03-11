# sagittarius_py_demo

Sagittarius Python 演示功能包 - 运动规划、视觉检测与抓取

## 功能描述

该功能包提供丰富的 Python 示例，涵盖运动规划、正逆运动学、视觉检测与抓取等核心功能：

### 运动规划
- **motion_planning_api_demo**: MoveIt 运动规划 API 演示
- **simple_fk_planning**: 正向运动学 (FK) 演示
- **simple_ik_planning**: 逆向运动学 (IK) 演示

### HSV 视觉检测与抓取
- **hsv_detect**: HSV 颜色检测
- **hsv_catch**: HSV 颜色引导抓取
- **ros_roi_hsv**: ROI 区域 HSV 检测
- **eye_out_hsv_detect**: 手眼外参 HSV 检测
- **eye_out_hsv_catch**: 手眼外参 HSV 引导抓取

### YOLO 视觉检测与抓取
- **yolo_catch**: YOLO 检测引导抓取
- **sag_detect**: Sagittarius 专用检测
- **sag_eye_out_motion**: 手眼外参运动规划

### 综合应用
- **meal_catch**: 餐品抓取综合应用

## 依赖

### 系统依赖
- Python 3.8+
- OpenCV
- NumPy

### Python 依赖
- `numpy`
- `opencv-python`
- `PyYAML`
- `cv_bridge` (ROS2)
- `transforms3d`

### ROS 2 依赖
- `rclpy`
- `moveit_python` / `moveit_py`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `vision_msgs`

### 构建工具
- `ament_python`

## 构建与安装

```bash
cd ~/sagittarius_humble_ws
source install/setup.bash
colcon build --packages-select sagittarius_py_demo
```

## 启动方式

### 运动规划示例

```bash
# MoveIt 运动规划 API 演示
ros2 launch sagittarius_py_demo motion_planning.launch.py

# 正向运动学演示
ros2 launch sagittarius_py_demo simple_fk_planning.launch.py

# 逆向运动学演示
ros2 launch sagittarius_py_demo simple_ik_planning.launch.py
```

### HSV 视觉示例

```bash
# HSV 颜色检测
ros2 run sagittarius_py_demo hsv_detect

# HSV 引导抓取
ros2 launch sagittarius_py_demo hsv_catch.launch.py

# ROI HSV 检测
ros2 launch sagittarius_py_demo hsv_roi.launch.py

# 手眼外参 HSV 检测
ros2 run sagittarius_py_demo eye_out_hsv_detect

# 手眼外参 HSV 抓取
ros2 launch sagittarius_py_demo eye_out_hsv_catch.launch.py
```

### YOLO 视觉示例

```bash
# YOLO 抓取
ros2 launch sagittarius_py_demo yolo_catch.launch.py

# 餐品抓取
ros2 launch sagittarius_py_demo yolo_meal_catch.launch.py
```

## 关键文件

| 文件路径 | 说明 |
|---------|------|
| `sagittarius_py_demo/motion_planning_api_demo.py` | 运动规划 API 演示 |
| `sagittarius_py_demo/simple_fk_planning.py` | 正向运动学演示 |
| `sagittarius_py_demo/simple_ik_planning.py` | 逆向运动学演示 |
| `sagittarius_py_demo/hsv_detect.py` | HSV 颜色检测 |
| `sagittarius_py_demo/hsv_catch.py` | HSV 引导抓取 |
| `sagittarius_py_demo/ros_roi_hsv.py` | ROI HSV 检测 |
| `sagittarius_py_demo/eye_out_hsv_detect.py` | 手眼外参 HSV 检测 |
| `sagittarius_py_demo/eye_out_hsv_catch.py` | 手眼外参 HSV 抓取 |
| `sagittarius_py_demo/yolo_catch.py` | YOLO 引导抓取 |
| `sagittarius_py_demo/sag_detect.py` | Sagittarius 检测 |
| `sagittarius_py_demo/sag_eye_out_motion.py` | 手眼外参运动规划 |
| `sagittarius_py_demo/meal_catch.py` | 餐品抓取应用 |
| `launch/motion_planning.launch.py` | 运动规划启动文件 |
| `launch/yolo_catch.launch.py` | YOLO 抓取启动文件 |
| `launch/yolo_meal_catch.launch.py` | 餐品抓取启动文件 |
| `rviz/` | RViz 配置目录 |

## 配置文件

配置文件位于 `params/` 目录，包含各功能模块的参数配置。

## RViz 配置

RViz 配置文件位于 `rviz/` 目录，用于可视化配置。
