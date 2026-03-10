# Sagittarius Vision 模块分析报告

> 分析日期: 2026-03-10

---

## 一、Sag_Vision 目录概览

| 包名 | 类型 | 功能 |
|------|------|------|
| **apriltag** | C++ 库 | AprilTag 视觉基准点检测核心库 |
| **apriltag_ros** | ROS2 封装 | 包含 apriltag_ros (节点) 和 apriltag_msgs (消息) |
| **ros2_yolov8** | Python | YOLOv8 目标检测/分割/姿态估计 |
| **realsense2_description** | URDF/Xacro | Intel RealSense 相机模型描述 |
| **usb_cam** | C++ | V4L USB 相机驱动 |
| **rs_camera_calibration** | 工具 | RealSense 相机标定工具 |

---

## 二、Apriltag 相关包分析

### 2.1 apriltag (库)
- **版本**: 3.4.2
- **类型**: CMake C++ 库
- **功能**: AprilTag 视觉基准点检测的核心算法库
- **特性**:
  - 支持多种标签家族 (tagCircle21h7 等)
  - 跨平台 (含 pthreads 封装)
  - 包含图像处理、边缘检测、姿态估计等模块

### 2.2 apriltag_ros
包含两个子包：

#### apriltag_msgs
- **版本**: 0.0.0
- **功能**: 定义 AprilTag 检测结果的消息类型
- **依赖**: geometry_msgs, std_msgs

#### apriltag_ros
- **版本**: 2.1.0
- **功能**: ROS2 AprilTag 检测节点
- **订阅**: 图像话题 (`/image_raw`)
- **发布**: 
  - `detections` - AprilTag 位置和姿态
  - `tf` - 标签的坐标变换

---

## 三、ros2_yolov8 包分析

| 项目 | 说明 |
|------|------|
| **类型** | Python (ament_python) |
| **功能** | YOLOv8 目标检测/实例分割/姿态估计 |

### 核心文件

| 文件 | 功能 |
|------|------|
| `ros2_object_detection.py` | 基础目标检测 |
| `ros2_pose_detection.py` | 人体姿态估计 |
| `ros2_instance_segmentation.py` | 实例分割 |
| `straight_obb.py` | **定制版**：带深度感知的 OBB 检测 |
| `ros2_transportation_detection.py` | 运输物体检测 |

### straight_obb.py 定制功能
- **订阅话题**:
  - RGB: `/camera/camera/color/image_raw`
  - 深度: `/camera/camera/aligned_depth_to_color/image_raw`
  - 相机内参: `/camera/camera/color/camera_info`
- **发布**: TF 变换、检测结果

### 模型文件
- `yolov8n.pt` - Nano 版本 (6.5MB)
- `config/fruits-obb.pt` - 定制OBB模型

---

## 四、realsense2_description 包

| 项目 | 说明 |
|------|------|
| **版本** | 4.51.1 |
| **类型** | URDF/Xacro 描述包 |
| **功能** | Intel RealSense D400 系列相机的 URDF 模型 |

### 支持的相机型号

| 型号 | 说明 |
|------|------|
| D415 | RGB-D 结构光相机 |
| D435 | RGB-D 深度相机 |
| D435i | 带 IMU 的 D435 |
| D455 | 双目深度相机 |
| L515 | LiDAR 深度相机 |

### 定制文件
- `leo_d435_camera.urdf.xacro` - Leo 机械臂 D435 配置
- `leo_arm_d435_camera.urdf.xacro` - 机械臂安装位姿

---

## 五、usb_cam 包

| 项目 | 说明 |
|------|------|
| **版本** | 0.8.1 |
| **类型** | C++ (ament_cmake) |
| **功能** | V4L (Video4Linux) USB 相机驱动 |

### 核心功能
- 从 V4L 设备读取图像
- 支持多种格式 (MJPG, YUYV, etc.)
- 发布: `~/image_raw` (sensor_msgs/Image)

---

## 六、rs_camera_calibration 包

| 项目 | 说明 |
|------|------|
| **功能** | RealSense 相机标定工具 |
| **依赖** | moveit_ros_planning_interface, moveit_visual_tools |

### 标定板配置
- Charuco 标定板: 14x9 格, 20mm 格子
- 图像话题: `/camera/camera/color/image_raw`

---

## 七、视觉模块数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                      视觉数据流                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   USB Camera / RealSense                                         │
│          ↓                                                       │
│   /usb_cam/image_raw (sensor_msgs/Image)                        │
│          ↓                                                       │
│   ┌─────────────────────────────────────┐                       │
│   │         视觉处理节点                  │                       │
│   │  • apriltag_detector                │                       │
│   │  • yolo_detector (OBB)              │                       │
│   │  • hsv_detect                       │                       │
│   └─────────────────────────────────────┘                       │
│          ↓                                                       │
│   检测结果 / TF / 目标位置 (geometry_msgs/Pose)                 │
│          ↓                                                       │
│   MoveIt 规划 + 机械臂控制                                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 八、应用场景

| 场景 | 使用模块 |
|------|----------|
| 颜色识别抓取 | usb_cam + HSV 检测 |
| 目标识别抓取 | ros2_yolov8 (OBB) |
| 视觉定位/标定 | apriltag_ros |
| 深度感知抓取 | realsense2_description |
| 手眼标定 | apriltag + rs_camera_calibration |

---

## 九、关键话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | USB 相机原始图像 |
| `/camera/color/image_raw` | sensor_msgs/Image | RealSense 彩色图像 |
| `/camera/depth/image_raw` | sensor_msgs/Image | RealSense 深度图像 |
| `/detections` | yolov8_ros_msgs/Detections | YOLO 检测结果 |
| `/tag_detections` | apriltag_ros_msgs/AprilTagDetectionArray | AprilTag 检测结果 |

---

*该报告由 Richard Agent 生成于 2026-03-10*
