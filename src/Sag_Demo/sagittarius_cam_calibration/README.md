# sagittarius_cam_calibration

相机标定功能包 - 支持 Charuco、AprilTag 等棋盘格标定

## 功能描述

该功能包提供多种相机标定方法，适用于工业相机、深度相机等设备的标定：

- **camera_calibration**: 相机标定主程序 (PyQt6 GUI)
- **charuco_detect**: Charuco 棋盘格检测
- **charuco_detect_tf**: 带 TF 变换的 Charuco 检测
- **cam2apriltag**: AprilTag 标定板检测与转换

## 依赖

### 系统依赖
- Python 3.8+
- OpenCV
- NumPy
- SciPy
- PyQt6

### Python 依赖
- `numpy`
- `opencv-python` / `opencv-contrib-python`
- `scipy`
- `PyQt6`
- `PyYAML`
- `cv_bridge` (ROS2)

### ROS 2 依赖
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `image_transport`

### 构建工具
- `ament_python`

## 构建与安装

```bash
cd ~/sagittarius_humble_ws
source install/setup.bash
colcon build --packages-select sagittarius_cam_calibration
```

## 启动方式

### 相机标定 (GUI)
```bash
ros2 run sagittarius_cam_calibration camera_calibration
```

### Charuco 检测
```bash
ros2 run sagittarius_cam_calibration charuco_detect
```

### Charuco 检测 (带 TF)
```bash
ros2 run sagittarius_cam_calibration charuco_detect_tf
```

### AprilTag 标定
```bash
ros2 run sagittarius_cam_calibration cam2apriltag
```

### 启动文件

```bash
# Roselle 相机标定
ros2 launch sagittarius_cam_calibration sag_rs_camera_calibration.launch.py

# AprilTag 标定
ros2 launch sagittarius_cam_calibration sag_apriltag_camera_calibration.launch.py

# Aruco 检测
ros2 launch sagittarius_cam_calibration sag_aruco_detect.launch.py

# Aruco 检测 (带 TF)
ros2 launch sagittarius_cam_calibration sag_aruco_detect_tf.launch.py
```

## 关键文件

| 文件路径 | 说明 |
|---------|------|
| `sagittarius_cam_calibration/camera_calibration.py` | 相机标定主程序 (GUI) |
| `sagittarius_cam_calibration/charuco_detect.py` | Charuco 检测 |
| `sagittarius_cam_calibration/charuco_detect_tf.py` | Charuco 检测 (TF) |
| `sagittarius_cam_calibration/cam2apriltag.py` | AprilTag 标定 |
| `launch/sag_rs_camera_calibration.launch.py` | 相机标定启动文件 |
| `launch/sag_apriltag_camera_calibration.launch.py` | AprilTag 标定启动文件 |
| `config/` | 配置文件目录 |

## 配置文件

配置文件位于 `config/` 目录，包含相机内参、标定参数等配置。
