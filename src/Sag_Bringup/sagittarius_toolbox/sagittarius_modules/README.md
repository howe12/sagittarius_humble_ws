# sagittarius_modules

ROS2 Sagittarius 机器人通用工具模块，提供机器人的公共接口和工具函数。

## 功能描述

该包提供 Sagittarius 机器人的通用工具模块：

- **机器人模型管理**：定义支持的机器人型号列表
- **关节配置**：获取各型号机器人的关节名称列表
- **Launch 工具**：提供机器人启动相关的辅助功能
- **公共接口**：封装机器人通用的操作接口

## 依赖

- `ament_python` - Python 构建类型
- `launch` - ROS2 启动系统
- `launch_ros` - ROS2 启动封装
- `xacro` - URDF 预处理器
- `tf_transformations` - TF 坐标变换库

## 启动方式

这是一个库包，不包含独立运行的节点。主要通过其他包导入使用：

```python
# 在其他 Python 代码中导入
from sagittarius_modules.sgr_common import get_sagittarius_arm_models
from sagittarius_modules.sgr_common import get_sagittarius_arm_joints
```

## 关键 API

### 获取支持的机器人型号

```python
from sagittarius_modules.sgr_common import get_sagittarius_arm_models

models = get_sagittarius_arm_models()
# 返回: ('sgr532',)
```

### 获取机器人关节列表

```python
from sagittarius_modules.sgr_common import get_sagittarius_arm_joints

joints = get_sagittarius_arm_joints('sgr532')
# 返回: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint_gripper_left', 'joint_gripper_right']
```

## 关键文件

```
sagittarius_modules/
├── sagittarius_modules/
│   ├── __init__.py
│   ├── sgr_common/
│   │   ├── __init__.py
│   │   └── sgr_common.py       # 公共工具函数
│   └── sgr_launch/
│       └── (启动相关工具)
├── setup.py
└── package.xml
```

## 支持的机器人型号

| 型号 | 描述 | 关节数 |
|------|------|--------|
| sgr532 | Sagittarius 532 机械臂 | 8 (6轴 + 2夹爪) |

---
Maintainer: sagittarius (litian.zhuang@nxrobo.com)
License: TODO
