# ros2_yolov8

YOLOv8 ROS 2 目标检测与实例分割包

## 功能描述

基于 Ultralytics YOLOv8 的 ROS 2 包，提供实时目标检测、实例分割和人体姿态估计功能。

主要功能：
- **目标检测**: 检测图像中的物体类别和位置
- **实例分割**: 像素级分割物体
- **姿态估计**: 检测人体关键点
- 支持图片和视频流处理

## 依赖

### Python 依赖
- `ultralytics` - YOLOv8 库
- `opencv-python` - 图像处理
- `numpy` - 数值计算
- `torch` / `torchvision` - 深度学习框架 (可选)

## 启动方式

### 1. 目标检测
```bash
ros2 run ros2_yolov8 ros2_object_detection
```

### 2. 实例分割
```bash
ros2 run ros2_yolov8 ros2_instance_segmentation
```

### 3. 姿态估计
```bash
ros2 run ros2_yolov8 ros2_pose_detection
```

### 4. 叉车托盘检测 (Transportation)
```bash
ros2 run ros2_yolov8 ros2_transportation_detection
```

### 5. YOLO 预测 (TensorFlow)
```bash
ros2 run ros2_yolov8 yolo_predict_tf
```

### 6. 水果检测
```bash
ros2 run ros2_yolov8 fruits_predict_tf
```

### 7. 餐饮检测
```bash
ros2 run ros2_yolov8 meal_predict_tf
```

### 8. 定向目标检测 (OBB)
```bash
ros2 run ros2_yolov8 straight_obb
```

## 关键文件

### 源码文件
| 文件 | 说明 |
|------|------|
| `ros2_yolov8/ros2_object_detection.py` | 目标检测节点 |
| `ros2_yolov8/ros2_instance_segmentation.py` | 实例分割节点 |
| `ros2_yolov8/ros2_pose_detection.py` | 姿态估计节点 |
| `ros2_yolov8/ros2_transportation_detection.py` | 运输场景检测 |
| `ros2_yolov8/yolo_predict_tf.py` | TensorFlow 版本 |
| `ros2_yolov8/fruits_predict_tf.py` | 水果检测 |
| `ros2_yolov8/meal_predict_tf.py` | 餐饮检测 |
| `ros2_yolov8/straight_obb.py` | 定向边界框 |

### 配置文件
| 目录 | 说明 |
|------|------|
| `config/` | 配置文件目录 |

### 模型文件
| 文件 | 说明 |
|------|------|
| `ros2_yolov8/yolov8n.pt` | YOLOv8 nano 检测模型 |
| `ros2_yolov8/yolov8n-seg.pt` | YOLOv8 nano 分割模型 |
| `ros2_yolov8/yolov8n-pose.pt` | YOLOv8 nano 姿态模型 |
| `ros2_yolov8/best.pt` | 自定义训练模型 |

## 输入/输出

### 输入
- `image_raw` 或 `/image` 话题 (sensor_msgs/Image)

### 输出
- 检测结果话题 (根据节点类型不同)
- 可视化图像

## 模型选择

```python
# YOLOv8 模型变体
- yolov8n.pt   # Nano (最快，精度最低)
- yolov8s.pt   # Small
- yolov8m.pt   # Medium
- yolov8l.pt   # Large
- yolov8x.pt   # XLarge (最慢，精度最高)
```

## 使用示例

```python
from ros2_yolov8 import YOLOv8

# 初始化
model = YOLOv8("yolov8n.pt", conf=0.25, iou=0.45)

# 检测
results = model.detect(frame)
```
