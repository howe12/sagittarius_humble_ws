# Sagittarius Demo 功能分析报告

> 分析日期: 2026-03-10

---

## 一、Sag_Demo 目录概览

`/home/leo/Music/sagittarius_humble_ws/src/Sag_Demo` 包含 **4 个 ROS2 包**，均为 Sagittarius 机械臂的演示应用。

---

## 二、各包详细分析

### 1. cobot_draw - 机械臂绘画功能包

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_cmake (C++) |
| **依赖** | moveit_core, moveit_ros_planning_interface, geometry_msgs |
| **功能** | 使用 MoveIt 控制机械臂进行绘画/绘图 |

**源文件：**
| 文件 | 功能 |
|------|------|
| `draw_circle.cpp` | 绘制圆形 - 通过计算圆周路径点，使用笛卡尔路径规划执行 |
| `draw_square.cpp` | 绘制正方形 |
| `draw_five_stars.cpp` | 绘制五角星 - 根据五角星几何顶点生成路径 |
| `draw_gcode.cpp` | 读取 G-code 文件执行绘图（支持 NC 格式） |
| `get_current_pose.cpp` | 获取机械臂当前末端位姿 |

**配置文件 (`config/`):**
- `aubo_60pt.nc` - G-code 刀具路径文件
- `five_pointed_star.nc` - 五角星 G-code
- `good.nc` - 通用 G-code 图形
- `config.yaml` - 绘图配置

**Launch 文件：**
- `draw_circle_true.launch.py` - 圆形绘画启动
- `draw_five_stars.launch.py` / `draw_five_stars_true.launch.py` - 五角星绘画
- `draw_square_true.launch.py` - 正方形绘画
- `draw_gcode.launch.py` - G-code 绘画启动

---

### 2. sagittarius_py_demo - Python 演示包

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_python |
| **功能** | 提供多种 Python 版本的机械臂演示（运动规划、视觉抓取等） |

**核心演示脚本：**

| 脚本 | 功能描述 |
|------|----------|
| `motion_planning_api_demo.py` | MoveIt 运动规划 API 演示，使用 MoveItPy |
| `simple_fk_planning.py` | 正向运动学 (Forward Kinematics) 演示 |
| `simple_ik_planning.py` | 逆向运动学 (Inverse Kinematics) 演示 |
| `eye_out_hsv_detect.py` | 外部摄像头 HSV 颜色检测 |
| `eye_out_hsv_catch.py` | 基于 HSV 颜色检测的视觉抓取 |
| `hsv_detect.py` | HSV 颜色空间目标检测 |
| `hsv_catch.py` | HSV 视觉引导抓取 |
| `ros_roi_hsv.py` | ROI 区域 HSV 检测 |
| `sag_detect.py` | Sagittarius 机械臂目标检测 |
| `sag_eye_out_motion.py` | 视觉引导的机械臂运动 |
| `yolo_catch.py` | YOLO 目标检测 + 机械臂抓取 |
| `meal_catch.py` | 餐食/物体抓取演示 |

---

### 3. sagittarius_humble_demo - Humble 版演示包

| 项目 | 说明 |
|------|------|
| **包名** | `sagittarius_puppet_control` |
| **构建类型** | ament_cmake |
| **功能** | 木偶控制 - 记录/回放机械臂运动轨迹 |

**源文件：**
| 文件 | 功能 |
|------|------|
| `puppet_control_single_node.cpp` | 单节点木偶控制 - 订阅关节状态，发布关节命令实现跟随控制 |
| `ros2bag_record_control.cpp` | 使用 ros2bag 记录/回放控制数据 |

**依赖：** sensor_msgs, std_msgs, sagittarius_common_msgs, rosbag2_transport

---

### 4. sagittarius_cam_calibration - 相机标定包

| 项目 | 说明 |
|------|------|
| **构建类型** | ament_python |
| **功能** | 相机标定与视觉定位 |

**核心脚本：**

| 脚本 | 功能描述 |
|------|----------|
| `camera_calibration.py` | 相机标定 - 使用 PyQt6 GUI + OpenCV ArUco 进行相机内参标定 |
| `charuco_detect.py` | Charuco 板检测 |
| `charuco_detect_tf.py` | Charuco 板 + TF 坐标变换检测 |
| `cam2apriltag.py` | 相机到 AprilTag 标定 - 获取相机与机械臂基座标间的外参 |

---

## 三、功能汇总表

| 包名 | 功能分类 | 技术栈 | 用途 |
|------|----------|--------|------|
| `cobot_draw` | 🎨 轨迹规划 | C++ / MoveIt | 机械臂绘图（圆、方、星、G-code） |
| `sagittarius_py_demo` | 🤖 运动控制 | Python / MoveItPy | 运动规划、视觉抓取（HSV/YOLO） |
| `sagittarius_puppet_control` | 🎬 轨迹录制 | C++ / rosbag | 木偶控制/轨迹复现 |
| `sagittarius_cam_calibration` | 📷 视觉标定 | Python / OpenCV | 相机内参标定、AprilTag 外参标定 |

---

## 四、技术特点

1. **cobot_draw**: 使用笛卡尔路径规划 (computeCartesianPath)，支持 G-code 解析
2. **sagittarius_py_demo**: 集成 OpenCV 视觉（HSV/YOLO）与 MoveIt 运动规划
3. **sagittarius_puppet_control**: 通过关节状态订阅实现机械臂跟随控制
4. **sagittarius_cam_calibration**: 结合 ArUco/Charuco/AprilTag 进行手眼标定

---

*该报告由 Ross Agent 生成于 2026-03-10*
