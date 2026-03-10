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
from ament_index_python.packages import get_package_share_directory


class CamHSVDetect(Node):
    def __init__(self):
        # 调用父类的初始化方法，设置节点名称为'RobotMoveitAction'
        super().__init__('CamHSVDetect')
        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("cam_hsv_detect") 
        # 创建Opencv转换器
        self.bridge = CvBridge()
        # 创建相机信息话题订阅者
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.imageCameraInfoCallback, 10)
        # 创建深度图像话题订阅者
        self.depth_sub = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.camera_depth_cb, 1)
        # 创建RGB图像话题订阅者
        self.image_subscriber = self.create_subscription(Image,  '/camera/camera/color/image_raw', self.image_callback, 10)
        # 创建定时器，用于周期性调用self.timer_callback函数
        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        sagittarius_py_demo_dir = sys.path[0]
        usb_cam_config_file = os.path.join(sagittarius_py_demo_dir, '../../share/sagittarius_py_demo/params', 'hsv_values.txt')

        # 💡 初始化参数
        self.transform_stamped = None
        self.grasp_flag = True
        self.min_rect = None
        # self.red_hsv    = [20, 180, 180] 
        # self.yellow_hsv = [25, 114, 231] 
        # self.green_hsv  = [85 , 255 , 138] 
        # self.blue_hsv   = [98 , 254 , 251] 

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
        # 💡 初始化HSV阈值范围   
        thresh = 25
        
        self.red_minHSV = np.array([self.red_hsv[0], self.red_hsv[1] - thresh, self.red_hsv[2] - thresh])
        self.red_maxHSV = np.array([self.red_hsv[0] + thresh, self.red_hsv[1] + thresh, self.red_hsv[2] + thresh])

        self.blue_minHSV = np.array([self.blue_hsv [0], self.blue_hsv [1]-thresh, self.blue_hsv [2]-thresh])
        self.blue_maxHSV = np.array([self.blue_hsv [0]+thresh, self.blue_hsv [1]+thresh, self.blue_hsv [2]+ thresh])

        self.yellow_minHSV = np.array([self.yellow_hsv[0] , self.yellow_hsv[1] - thresh, self.yellow_hsv[2] - thresh])
        self.yellow_maxHSV = np.array([self.yellow_hsv[0] + thresh, self.yellow_hsv[1] + thresh, self.yellow_hsv[2] + thresh])
   
        self.green_minHSV = np.array([self.green_hsv[0] , self.green_hsv[1] - thresh, self.green_hsv[2] - thresh])
        self.green_maxHSV = np.array([self.green_hsv[0] + thresh, self.green_hsv[1] + thresh, self.green_hsv[2] + thresh])
   
        time.sleep(2)


    def imageCameraInfoCallback(self, msg):
        """
        获取相机内参信息
        """
        # 获取图像尺寸
        self.image_width = msg.width
        self.image_height = msg.height
        # 获取相机内参
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
    
    def load_hsv_values(self,filepath):
        with open(filepath, 'r') as file:
            hsv_data = json.load(file)
        return hsv_data

    def camera_depth_cb(self, msg):
        """
        深度图像回调函数
        """
        # 将ROS2图像消息转换为OpenCV格式
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")


    def get_valid_depth(self,x, y, search_radius=5):
        """
        获取深度图像中指定坐标点的有效深度值。

        参数:
        x (int): 像素点的 x 坐标。
        y (int): 像素点的 y 坐标。
        search_radius (int): 搜索半径，默认为 5。

        返回:
        float: 有效深度值，如果没有找到有效深度值，则返回 0。
        """
        # 判断是否有深度图像
        if self.depth_image is None:
            return
        # 获取深度图像的高度和宽度
        h, w = self.depth_image.shape
        # 初始化统计深度次数
        count = 0
        sum_z = 0.0
        # 遍历搜索半径内的所有像素点
        for r in range(1, 5):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    # 计算当前像素点的坐标
                    nx, ny = x + dx, y + dy
                    # 如果当前像素点在深度图像范围内
                    if 0 <= nx < w and 0 <= ny < h:  
                        # 获取当前像素点的深度值
                        z = self.depth_image[ny, nx] / 1000.0  
                        # 如果当前像素点的深度值有效, 则将其加入统计深度次数
                        if z > 0:  
                            sum_z += z
                            count += 1
        # 如果在搜索半径内找到了有效深度值，则返回平均深度值
        if count > 0:
            return sum_z / count
        else:
            # 如果在搜索半径内没有找到有效深度值，则返回 0
            return 0


    # 图像回调函数
    def image_callback(self, msg):
        """
        RGB图像回调函数
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 将ros2图像格式转化成opencv格式
        hsv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # 由RGB颜色转换成HSV颜色空间
    
        mask_red = cv2.inRange(hsv_img, self.red_minHSV , self.red_maxHSV)
        mask_blue = cv2.inRange(hsv_img, self.blue_minHSV, self.blue_maxHSV )
        mask_yellow = cv2.inRange(hsv_img, self.yellow_minHSV, self.yellow_maxHSV)
        mask_green = cv2.inRange(hsv_img, self.green_minHSV, self.green_maxHSV)

        # 检测物体中心点位置并发布物体的坐标
        self.detect_mid_pub_tf(mask_blue,"blue") 
        self.detect_mid_pub_tf(mask_yellow,"yellow")
        self.detect_mid_pub_tf(mask_green,"green")

        # 合并所有颜色的掩码
        mask_combined = cv2.bitwise_or(mask_green, mask_blue)
        resultHSV = cv2.bitwise_or(mask_combined, mask_yellow)

        # cv2.imshow("mask_yellow", mask_yellow)
        # cv2.imshow("mask_green", mask_green)
        # cv2.imshow("mask_blue", mask_blue)
        cv2.imshow("ResultHSV", resultHSV)
        cv2.waitKey(1)


    def detect_mid_pub_tf(self,img,color):
        """
        检测物体并发布物体的坐标
        """
        # 寻找物体的轮廓
        contours, hier = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 2)

        # 获取水平矩形/最小矩形
        for contour in contours:
            area = cv2.contourArea(contour)
            # print("area=",area)
            
            if area < 600: # 排除误识别小范围颜色
                continue
            self.min_rect = cv2.minAreaRect(contour)  #最小矩形
            
        # 若self.min_rect为空则退出
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

        # 计算物体相对与相机的距离
        dis_z = self.get_valid_depth(m_x,m_y)

        # 排除误识别
        if dis_z == 0:
            return

        # 计算成像坐标
        x_cam = (pixel_x - self.cx) * dis_z / self.fx
        y_cam = (pixel_y - self.cy) * dis_z / self.fy

        cube_tf = TransformStamped()
        cube_tf.header.stamp = self.get_clock().now().to_msg()
        cube_tf.header.frame_id = "camera_link"
        cube_tf.child_frame_id = str(color)+"_object"
        cube_tf.transform.translation.x = float(dis_z) # float(point_z)
        cube_tf.transform.translation.y = -float(x_cam) # -float(point_x)
        cube_tf.transform.translation.z = -float(y_cam) # -float(point_y)
        ori = tf_transformations.quaternion_from_euler(theta+ 3.1416 , 0, 0)
        cube_tf.transform.rotation.x = ori[0]
        cube_tf.transform.rotation.y = ori[1]
        cube_tf.transform.rotation.z = ori[2]
        cube_tf.transform.rotation.w = ori[3]
        self.tf_pub.sendTransform(cube_tf)
        # time.sleep(2)
        self.min_rect = None
        cv2.imshow("ResultRGB", self.cv_image)
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