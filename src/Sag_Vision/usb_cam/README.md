# usb_cam

USB 摄像头 ROS 2 驱动

## 功能描述

V4L (Video4Linux) USB 摄像头驱动，提供标准的 ROS 2 图像话题发布。

主要功能：
- 支持 V4L 兼容的 USB 摄像头
- 发布标准 sensor_msgs/Image 话题
- 支持多种视频格式 (MJPEG, YUYV, RGB24 等)
- 相机内参管理
- 视频参数可配置

## 依赖

### 核心依赖
- `rclcpp` - ROS 2 C++ 客户端
- `rclcpp_components` - 组件系统
- `sensor_msgs` - 传感器消息
- `std_msgs` - 标准消息
- `std_srvs` - 标准服务
- `cv_bridge` - OpenCV 桥接
- `image_transport` - 图像传输
- `camera_info_manager` - 相机信息管理
- `v4l-utils` - V4L 工具

### 可选依赖
- `ffmpeg` - MJPEG 到 RGB 转换

## 启动方式

### 1. 启动默认配置
```bash
ros2 launch usb_cam camera.launch.py
```

### 2. 指定参数启动
```bash
ros2 launch usb_cam camera.launch.py device:=/dev/video0
```

### 3. 程序方式启动
```bash
ros2 run usb_cam usb_cam_node_exe
```

## 关键文件

### Launch 文件
| 文件 | 说明 |
|------|------|
| `launch/camera.launch.py` | 相机启动配置 |

### 配置目录
| 文件 | 说明 |
|------|------|
| `config/` | 相机配置文件 |

### 源码
| 文件 | 说明 |
|------|------|
| `src/usb_cam.cpp` | 主节点实现 |
| `src/usb_cam_node.cpp` | 节点入口 |

### 消息定义
| 文件 | 说明 |
|------|------|
| `msg/ExclusionZoneItem.msg` | 排除区域消息 |
| `msg/RegionOfInterestInt.msg` | ROI 消息 |

### 服务
| 文件 | 说明 |
|------|------|
| `srv/Start.srv` | 启动服务 |
| `srv/Stop.srv` | 停止服务 |

## 参数配置

### 常用参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `device` | string | `/dev/video0` | 视频设备 |
| `pixel_format` | string | `mjpeg` | 像素格式 |
| `image_width` | int | 640 | 图像宽度 |
| `image_height` | int | 480 | 图像高度 |
| `framerate` | int | 30 | 帧率 |
| `camera_name` | string | `usb_cam` | 相机名称 |
| `camera_info_url` | string | `` | 相机参数 URL |

### 像素格式
- `mjpeg` - Motion JPEG
- `yuyv` - YUV 4:2:2
- `rgb24` - RGB 24-bit
- `uyvy` - UYVY 格式

## 话题与服务

### 发布的话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 原始图像 |
| `/usb_cam/camera_info` | sensor_msgs/CameraInfo | 相机参数 |

### 服务
| 服务 | 类型 | 说明 |
|------|------|------|
| `/usb_cam/start` | usb_cam/Start | 启动采集 |
| `/usb_cam/stop` | usb_cam/Stop | 停止采集 |

## 使用示例

### 配置示例 (YAML)
```yaml
usb_cam_node:
  ros__parameters:
    device: /dev/video0
    pixel_format: mjpeg
    image_width: 1280
    image_height: 720
    framerate: 30
    camera_name: front_camera
```

### 配合 AprilTag 使用
```bash
# 启动 USB 相机
ros2 launch usb_cam camera.launch.py

# 启动 AprilTag 检测
ros2 launch apriltag_ros tag_usbcam.launch.py
```
