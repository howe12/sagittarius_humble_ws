# apriltag_ros

AprilTag ROS 2 检测节点

## 功能描述

基于 apriltag 库的 ROS 2 节点，提供 AprilTag 标签检测和姿态估计功能。

主要功能：
- 实时 AprilTag 检测
- 标签位姿发布 (TF + 消息)
- 支持多标签同时检测
- 可配置的标签家族和尺寸

## 依赖

### 核心依赖
- `rclcpp` - ROS 2 C++ 客户端
- `sensor_msgs` - 传感器消息
- `geometry_msgs` - 几何消息
- `tf2_ros` / `tf2_msgs` - TF 变换
- `image_transport` - 图像传输
- `image_geometry` - 相机模型
- `cv_bridge` - OpenCV 桥接
- `eigen` - 线性代数库
- `apriltag` - 检测库
- `apriltag_msgs` - AprilTag 消息

## 启动方式

### 1. 使用 USB Camera
```bash
ros2 launch apriltag_ros tag_usbcam.launch.py
```

### 2. 使用 RealSense 相机
```bash
ros2 launch apriltag_ros tag_realsense.launch.py
```

### 3. Gazebo 仿真
```bash
ros2 launch apriltag_ros tag_gazebo.launch.py
```

## 关键文件

### Launch 文件
| 文件 | 说明 |
|------|------|
| `apriltag_ros/launch/tag_usbcam.launch.py` | USB Camera 启动 |
| `apriltag_ros/launch/tag_realsense.launch.py` | RealSense 启动 |
| `apriltag_ros/launch/tag_gazebo.launch.py` | Gazebo 仿真 |

### 配置文件
| 文件 | 说明 |
|------|------|
| `apriltag_ros/cfg/tags_36h11_filter.yaml` | 36h11 标签配置 |

### 消息接口
- `apriltag_msgs/msg/AprilTagDetection` - 单个标签检测
- `apriltag_msgs/msg/AprilTagDetectionArray` - 多个标签检测

## 参数配置

常用参数：
- `tag_family`: 标签家族 (36h11, 25h9, 16h5 等)
- `tag_size`: 标签物理尺寸 (米)
- `camera_name`: 相机名称
- `image_topic`: 图像话题

## 标签坐标系

检测到的标签会发布：
1. **TF 变换**: 从 camera_frame 到 tag_<id> frame
2. **话题消息**: AprilTagDetectionArray 包含位姿信息
