# sagittarius_descriptions

ROS2 机器人描述包，提供 Sagittarius 机器人的 URDF 描述、配置和可视化文件。

## 功能描述

该包提供 Sagittarius 机器人的完整描述文件，用于：

- **机器人建模**：使用 URDF/Xacro 定义机器人运动学和动力学参数
- **可视化**：在 RViz 中显示机器人模型
- **仿真**：支持 Gazebo 仿真配置
- **状态发布**：使用 robot_state_publisher 发布机器人状态
- **关节控制**：支持 joint_state_publisher 进行关节控制

## 依赖

- `ament_cmake` - CMake 构建系统
- `robot_state_publisher` - 机器人状态发布器
- `joint_state_publisher` - 关节状态发布器
- `joint_state_publisher_gui` - 关节状态发布器 GUI
- `rviz2` - 3D 可视化工具
- `xacro` - URDF 预处理器
- `urdf` - URDF 工具库
- `sagittarius_modules` - Sagittarius 机器人模块

## 启动方式

### 方式 1：启动机器人描述

```bash
# 基础描述启动
ros2 launch sagittarius_descriptions sagittarius_description.launch.py

# Gazebo 仿真描述
ros2 launch sagittarius_descriptions sagittarius_gazebo_description.launch.py
```

### 方式 2：单独使用 xacro 生成 URDF

```bash
# 生成 URDF 文件
xacro ~/sagittarius_humble_ws/src/Sag_Bringup/sagittarius_descriptions/urdf/sgr532.urdf.xacro > /tmp/sgr532.urdf

# 在 RViz 中查看
ros2 run rviz2 rviz2 -f robot_description
```

## 关键文件

```
sagittarius_descriptions/
├── urdf/
│   ├── sgr532.urdf.xacro          # 主 URDF 定义 (Sgr532 机器人)
│   ├── control.urdf.xacro         # 控制器配置
│   ├── gz_control.urdf.xacro     # Gazebo 控制器配置
│   └── spark_sgr_external.urdf.xacro  # 外部接口配置
├── launch/
│   ├── sagittarius_description.launch.py       # 描述启动文件
│   └── sagittarius_gazebo_description.launch.py # Gazebo 描述启动
├── meshes/
│   └── (机器人3D模型文件)
├── rviz/
│   └── (RViz 配置文件)
├── config/
│   └── (参数配置文件)
├── CMakeLists.txt
└── package.xml
```

## 机器人模型

### Sgr532

Sagittarius Sgr532 是一款6轴机械臂，关节配置：

| 关节名称 | 类型 | 描述 |
|----------|------|------|
| joint1 | revolute | 底座旋转 |
| joint2 | revolute | 大臂 |
| joint3 | revolute | 小臂 |
| joint4 | revolute | 腕部旋转 |
| joint5 | revolute | 腕部俯仰 |
| joint6 | revolute | 末端旋转 |
| joint_gripper_left | prismatic | 夹爪左 |
| joint_gripper_right | prismatic | 夹爪右 |

## 使用示例

### 在 Python 中加载 URDF

```python
from urllib.parse import urljoin
from ament_index_python.packages import get_package_share_directory
import xacro

def load_robot_description():
    pkg_path = get_package_share_directory('sagittarius_descriptions')
    xacro_file = os.path.join(pkg_path, 'urdf', 'sgr532.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    return doc.toxml()
```

### 使用 robot_state_publisher

```bash
ros2 run robot_state_publisher robot_state_publisher /robot_description:=robot_description
```

---
Maintainer: sagittarius (litian.zhuang@nxrobo.com)
License: BSD
