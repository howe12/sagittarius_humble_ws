# apriltag

AprilTag 视觉标记检测库

## 功能描述

AprilTag 是一个视觉基准系统，用于机器人定位、AR 和相机标定。该库提供高效的 AprilTag 标签检测算法。

主要功能：
- AprilTag 标签检测 (36h11, 25h9, 16h5 等家族)
- 姿态估计
- 标签边角检测
- 支持多种标签家族

## 依赖

### 构建依赖
- `cmake` - 构建系统
- `python3-dev` (Python 3)
- `python3-numpy` (Python 3)

### 测试依赖
- `libopencv-dev` - OpenCV (仅用于测试)

## 构建方式

```bash
cd ~/sagittarius_humble_ws/src/Sag_Vision/apriltag
mkdir build && cd build
cmake ..
make
sudo make install
```

## 关键文件

| 文件 | 说明 |
|------|------|
| `CMakeLists.txt` | 构建配置 |
| `package.xml` | 包描述 |

## AprilTag 标签家族

支持的标签家族：
- **36h11**: 最常用，适合室内环境
- **25h9**: 较大标签，适合远距离
- **16h5**: 较小标签，适合近距离
- **41h12**: 高密度
- **Circle21h7**, **Circle49h12**: 圆形标签
- **Grid48h12**: 网格标签

## 相关包

- [apriltag_ros](./apriltag_ros) - ROS 2 集成包
- apriltag_msgs - AprilTag 消息定义
