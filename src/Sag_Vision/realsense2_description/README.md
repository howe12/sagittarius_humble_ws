# realsense2_description

Intel RealSense D400 系列相机 URDF/Xacro 模型包

## 功能描述

提供 Intel RealSense D400 系列深度相机的 URDF 描述和可视化模型，用于：
- MoveIt 运动规划中的相机模型
- Gazebo 仿真
- RViz 可视化

支持的相机型号：
- D415
- D435
- D435i
- D455
- D457

## 依赖

### 构建依赖
- `ament_cmake` - 构建系统
- `rclcpp` - ROS 2 客户端
- `rclcpp_components` - 组件系统

### 运行依赖
- `realsense2_camera_msgs` - RealSense 消息
- `xacro` - Xacro 预处理
- `launch_ros` - Launch 系统

## 启动方式

### RViz 可视化
```bash
ros2 launch realsense2_description view_model.launch.py
```

### 在 Xacro 中使用
```xacro
<!-- 包含 D435 相机模型 -->
<xacro:include filename="$(find realsense2_description)/urdf/d435.urdf.xacro" />

<!-- 使用相机 -->
<d435 parent="base_link">
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</d435>
```

## 关键文件

### URDF/Xacro 文件
| 文件 | 说明 |
|------|------|
| `urdf/d415.urdf.xacro` | D415 相机模型 |
| `urdf/d435.urdf.xacro` | D435 相机模型 |
| `urdf/d435i.urdf.xacro` | D435i 相机模型 (带 IMU) |
| `urdf/d455.urdf.xacro` | D455 相机模型 |

### Launch 文件
| 文件 | 说明 |
|------|------|
| `launch/view_model.launch.py` | RViz 模型可视化 |

### Mesh 文件
| 文件 | 说明 |
|------|------|
| `meshes/d415.stl` | D415 外观模型 |
| `meshes/d435.stl` | D435 外观模型 |
| `meshes/d455.stl` | D455 外观模型 |

## 相机参数

各型号相机参数：

| 型号 | RGB 分辨率 | 深度 FOV | 深度范围 |
|------|-----------|----------|----------|
| D415 | 1920x1080 | 65°×40° | 0.2-10m |
| D435 | 1920x1080 | 91°×65° | 0.2-10m |
| D435i | 同 D435 + IMU | 同 D435 | 0.2-10m |
| D455 | 1280×800 | 95°×70° | 0.4-20m |

## 使用场景

1. **机械臂末端集成**: 将相机安装在机械臂末端进行视觉引导
2. **固定式观测**: 作为环境感知传感器
3. **仿真**: 在 Gazebo 中使用相机模型
4. **手眼标定**: 与机械臂配合进行手眼标定
