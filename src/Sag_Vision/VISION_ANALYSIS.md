# Sagittarius Vision 模块分析报告

> 分析日期: 2026-03-10

---

## 一、Sag_Vision 目录概览

`/home/leo/Music/sagittarius_humble_ws/src/Sag_Vision` 包含机械臂视觉相关的 ROS2 包。

---

## 二、各包详细分析

### 1. apriltag_ros - AprilTag 视觉定位

| 项目 | 说明 |
|------|------|
| **功能** | AprilTag 标记检测与定位 |
| **依赖** | image_transport, sensor_msgs, geometry_msgs, tf2 |
| **用途** | 机器人视觉定位、标定、手眼协调 |

**关键节点：**
- `apriltag_detector` - AprilTag 检测

**支持标签：**
- Tag36h11, Tag25h9, Tag16h5 等多种标签家族

---

### 2. ros2_yolov8 - YOLOv8 目标检测

| 项目 | 说明 |
|------|------|
| **功能** | YOLOv8 实时目标检测 |
| **依赖** | sensor_msgs, image_transport, cv_bridge, ultralytics |
| **用途** | 机器人视觉抓取、目标识别 |

**特点：**
- 实时推理
- 支持自定义模型
- 可输出检测框和类别

**关键节点：**
- `yolo_detector` - YOLOv8 检测节点

---

### 3. realsense2_description - RealSense 相机描述

| 项目 | 说明 |
|------|------|
| **功能** | Intel RealSense 相机 URDF/Xacro 描述文件 |
| **依赖** | urdf, xacro |
| **用途** | 提供 RealSense 相机的 URDF 模型，用于仿真 |

**支持型号：**
- D415, D435, D455 等

---

### 4. usb_cam - USB 相机驱动

| 项目 | 说明 |
|------|------|
| **功能** | USB 相机驱动 |
| **依赖** | image_transport, sensor_msgs, cv_bridge, v4l2 |
| **用途** | 读取 USB 摄像头图像 |

**关键节点：**
- `usb_cam` - 相机驱动节点

**发布话题：**
- `/usb_cam/image_raw` - 原始图像

---

## 三、视觉模块数据流

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
│   │  • yolo_detector                    │                       │
│   │  • hsv_detect                       │                       │
│   └─────────────────────────────────────┘                       │
│          ↓                                                       │
│   检测结果 / 目标位置 (geometry_msgs/Pose)                       │
│          ↓                                                       │
│   MoveIt 规划 + 机械臂控制                                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 四、视觉相关话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | USB 相机原始图像 |
| `/camera/color/image_raw` | sensor_msgs/Image | RealSense 彩色图像 |
| `/camera/depth/image_raw` | sensor_msgs/Image | RealSense 深度图像 |
| `/detections` | yolov8_ros_msgs/Detections | YOLO 检测结果 |
| `/tag_detections` | apriltag_ros_msgs/AprilTagDetectionArray | AprilTag 检测结果 |

---

## 五、应用场景

| 场景 | 使用模块 |
|------|----------|
| 颜色识别抓取 | usb_cam + HSV 检测 |
| 目标识别抓取 | ros2_yolov8 |
| 视觉定位/标定 | apriltag_ros |
| 深度感知抓取 | realsense2_description |

---

*该报告由 Richard Agent 生成于 2026-03-10*
