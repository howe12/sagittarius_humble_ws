# Copyright (c) 2025 NXROBO
#
# /* Author: haijie.huo */
# /* email: haijie.huo@nxrobo.com */
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup

import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
from rclpy.time import Time
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

import json
import os
import sys


class CamHSVDetect(Node):
    def __init__(self):
        # 调用父类的初始化方法，设置节点名称为'RobotMoveitAction'
        super().__init__('CamHSVDetect')

        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("cam_hsv_detect") 

        # 创建Opencv转换器
        self.bridge = CvBridge()
        # 订阅相机信息话题
        self.image_topic = self.declare_parameter('image_topic', '/camera1/image_raw').value
        self.image_info_topic = self.declare_parameter('image_info_topic', '/camera1/camera_info').value
        self.sub_info = self.create_subscription(CameraInfo, self.image_info_topic, self.imageCameraInfoCallback, 10)
        # 创建图像订阅者
        self.image_subscriber = self.create_subscription(Image,  self.image_topic, self.image_callback, 10)

        sagittarius_py_demo_dir = sys.path[0]
        usb_cam_config_file = os.path.join(sagittarius_py_demo_dir, '../../share/sagittarius_py_demo/params', 'hsv_values.txt')

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.transform_stamped = None
        self.grasp_flag = True
        self.min_rect = None
        self.fx = 759.63
        self.fy = 760.08
        self.cx = 303.25
        self.cy = 230.05

        # 读取hsv.txt文件

        default_hsv = {
            "red": [20, 180, 180],
            "yellow": [60, 200, 180],
            "green": [120, 200, 180],
            "blue": [180, 200, 180]
        }

        # 读取参数并赋值
        hsv_values = self.load_hsv_values(usb_cam_config_file)

        self.red_hsv = hsv_values.get("red", default_hsv["red"])
        self.yellow_hsv = hsv_values.get("yellow", default_hsv["yellow"])
        self.green_hsv = hsv_values.get("green", default_hsv["green"])
        self.blue_hsv = hsv_values.get("blue", default_hsv["blue"])

        # 输出参数值
        self.get_logger().info("red: %s" % str(self.red_hsv))
        self.get_logger().info("yellow: %s" % str(self.yellow_hsv ))
        self.get_logger().info("green: %s" % str(self.green_hsv ))
        self.get_logger().info("blue: %s" % str(self.blue_hsv ))
        
        # h_thresh = 15
        thresh = 25

        # self.red_hsv = cv2.cvtColor(np.uint8([[red]]), cv2.COLOR_BGR2HSV)[0][0]
        # self.blue_hsv = cv2.cvtColor(np.uint8([[blue]]), cv2.COLOR_BGR2HSV)[0][0]
        # self.yellow_hsv = cv2.cvtColor(np.uint8([[yellow]]), cv2.COLOR_BGR2HSV)[0][0]
        # self.green_hsv = cv2.cvtColor(np.uint8([[green]]), cv2.COLOR_BGR2HSV)[0][0]


        self.red_minHSV = np.array([self.red_hsv[0], self.red_hsv[1] - thresh, self.red_hsv[2] - thresh])
        self.red_maxHSV = np.array([self.red_hsv[0] + thresh, self.red_hsv[1] + thresh, self.red_hsv[2] + thresh])

        self.blue_minHSV = np.array([self.blue_hsv [0], self.blue_hsv [1]-thresh, self.blue_hsv [2]-thresh])
        self.blue_maxHSV = np.array([self.blue_hsv [0]+thresh, self.blue_hsv [1]+thresh, self.blue_hsv [2]+ thresh])

        self.yellow_minHSV = np.array([self.yellow_hsv[0] , self.yellow_hsv[1] - thresh, self.yellow_hsv[2] - thresh])
        self.yellow_maxHSV = np.array([self.yellow_hsv[0] + thresh, self.yellow_hsv[1] + thresh, self.yellow_hsv[2] + thresh])
   
        self.green_minHSV = np.array([self.green_hsv[0] , self.green_hsv[1] - thresh, self.green_hsv[2] - thresh])
        self.green_maxHSV = np.array([self.green_hsv[0] + thresh, self.green_hsv[1] + thresh, self.green_hsv[2] + thresh])
   

        time.sleep(2)


    def load_hsv_values(self,filepath):
        with open(filepath, 'r') as file:
            hsv_data = json.load(file)
        return hsv_data

    def imageCameraInfoCallback(self, msg):
        # 获取图像尺寸
        self.image_width = msg.width
        self.image_height = msg.height
        # # 获取相机内参
        # self.fx = msg.k[0]
        # self.fy = msg.k[4]
        # self.cx = msg.k[2]
        # self.cy = msg.k[5]

        # 打印相机内参

        # self.logger.info("fx = %f, fy = %f, cx = %f, cy = %f", self.fx, self.fy, self.cx, self.cy)


    # 图像回调函数
    def image_callback(self, msg):
        # self.get_logger().info("Image received")
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 将ros2图像格式转化成opencv格式
        hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # 由RGB颜色转换成HSV颜色空间
        cv2.imshow("hsv_img", hsv_img)
        
        # # 蓝色物体颜色检测范围
        # LowerBlue = np.array([95, 150, 150])
        # UpperBlue = np.array([130, 255, 255])
        # # 阈值处理
        # mask = cv2.inRange(hsv_img, LowerBlue, UpperBlue)

        mask_red = cv2.inRange(hsv_img, self.red_minHSV , self.red_maxHSV)
        mask_blue = cv2.inRange(hsv_img, self.blue_minHSV, self.blue_maxHSV )
        mask_yellow = cv2.inRange(hsv_img, self.yellow_minHSV, self.yellow_maxHSV)
        mask_green = cv2.inRange(hsv_img, self.green_minHSV, self.green_maxHSV)

        # 合并所有颜色的掩码
        mask_combined = cv2.bitwise_or(mask_green, mask_blue)
        resultHSV = cv2.bitwise_or(mask_combined, mask_yellow)

        cv2.imshow("mask_yellow", mask_yellow)
        cv2.imshow("mask_green", mask_green)
        cv2.imshow("mask_blue", mask_blue)
        cv2.waitKey(1)

        # 寻找物体的轮廓
        contours, hier = cv2.findContours(resultHSV, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 2)

        # 获取水平矩形/最小矩形
        for contour in contours:
            area = cv2.contourArea(contour)
            # print("area=",area)
            
            if area < 18000: # 排除误识别
                continue
            # (x, y, w, h) = cv2.boundingRect(contour)  #水平矩形
            self.min_rect = cv2.minAreaRect(contour)  #最小矩形
            
        # 判断self.min_rect 是否为空
        if self.min_rect is None:
            return
        
        # 计算最小矩形的中心点
        pixel_x, pixel_y = self.min_rect[0]
        cv2.circle(self.cv_image, (int(pixel_x), int(pixel_y)), 3, (0, 255, 0), 5)
        # 绘制最小矩形
        width, height = self.min_rect[1]
        theta = self.min_rect[2]
        theta = math.radians(theta) # 将角度转换成弧度
        
        # cv2.polylines(img=self.cv_rgb, pts=[box], isClosed=True, color=(0, 255, 0), thickness=2)
        m_x, m_y, m_w, m_h = int(pixel_x), int(pixel_y), int(width), int(height)
        roi_img = self.cv_image[m_y: m_y+m_h, m_x: m_x+m_w]

        # 计算物体中心点距离图像中心的距离
        dis_x = math.sqrt((pixel_x- self.image_width/2)**2 + (pixel_y- self.image_height/2)**2)*(0.30/self.image_width)

        # 计算物体相对与相机的距离
        dis_z = math.sqrt(0.2**2 + dis_x**2) # 机械臂检测位置离台高度0.265,离物体平面0.2

        # 计算 成像坐标
        x_cam = (pixel_x - self.cx) * dis_z / self.fx
        y_cam = (pixel_y - self.cy) * dis_z / self.fy

        cube_tf = TransformStamped()
        cube_tf.header.stamp = self.get_clock().now().to_msg()
        cube_tf.header.frame_id = "sgr532/usb_cam_link"
        cube_tf.child_frame_id = "object"
        cube_tf.transform.translation.x = float(dis_z) # float(point_z)
        cube_tf.transform.translation.y = -float(x_cam) # -float(point_x)
        cube_tf.transform.translation.z = -float(y_cam) # -float(point_y)
        ori = tf_transformations.quaternion_from_euler(theta, 0, 0)
        cube_tf.transform.rotation.x = ori[0]
        cube_tf.transform.rotation.y = ori[1]
        cube_tf.transform.rotation.z = ori[2]
        cube_tf.transform.rotation.w = ori[3]
        self.tf_pub.sendTransform(cube_tf)
        # time.sleep(2)
        self.min_rect = None

        cv2.imshow("Image", self.cv_image)
        cv2.waitKey(1)


       
def main():
    rclpy.init()
    node = CamHSVDetect()
    # 使用 MultiThreadedExecutor 执行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()