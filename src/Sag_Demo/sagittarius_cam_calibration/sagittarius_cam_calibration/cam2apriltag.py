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

from cv2 import aruco
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

class Cam2Object(Node):
    def __init__(self):
        # 调用父类的初始化方法，设置节点名称为'RobotMoveitAction'
        super().__init__('Cam2Object')

        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("Cam_to_Object") 

        # 创建Opencv转换器
        self.bridge = CvBridge()
        # 创建图像订阅者
        self.image_subscriber = self.create_subscription(Image,  '/camera/camera/color/image_raw', self.image_callback, 10)
        # 创建机械臂位姿订阅者
        # self.pose_subscriber = self.create_subscription(PoseStamped, 'joint_states', self.pose_callback, 10)

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.fx = 759.63
        self.fy = 760.08
        self.cx = 303.25
        self.cy = 230.05

        self.mark_timer = 0

        self.readings = []

        self.R_target2cam = []
        self.t_target2cam = []
        self.R_base2gripper = []
        self.t_base2gripper = []

        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grasp_timer = self.create_timer(3, self.get_obj_trans)
        # self.get_logger().info("✅ 💡 定时器已创建，每 3 秒执行一次 get_trans()！")



    def get_obj_trans(self):
        
        time.sleep(1)
        
        # 获取物体坐标转换
        self.transform_stamped = None
        try:
            self.transform_stamped = self.tf_buffer.lookup_transform("camera_link", "object3",Time())
            
            # 1. 提取平移向量
            translation = [
                self.transform_stamped.transform.translation.x,
                self.transform_stamped.transform.translation.y,
                self.transform_stamped.transform.translation.z
            ]

            # 2. 提取四元数
            quaternion = [
                self.transform_stamped.transform.rotation.x,
                self.transform_stamped.transform.rotation.y,
                self.transform_stamped.transform.rotation.z,
                self.transform_stamped.transform.rotation.w
            ]

            # 3. 通过 scipy.spatial.transform.Rotation 转换为旋转矩阵
            rotation_matrix = R.from_quat(quaternion).as_matrix()

            # 4. 输出结果
            # self.get_logger().info(f"🎯 平移向量为: {translation}")
            # self.get_logger().info(f"🎯 旋转矩阵为: {rotation_matrix}")
            # self.get_logger().info(f"🚀 转换信息为: {self.transform_stamped}")
            return True, translation , rotation_matrix

        except TransformException as ex:
            self.get_logger().info(f'\033[33m ❌ Could not transform camera_link to object3: {ex}\033[0m')
            return False, None ,None
        
    def get_arm_trans(self):
        "获取机械臂位姿关系"
        
        time.sleep(1)
        
        # 获取物体坐标转换
        arm_transform_stamped = None
        try:
            arm_transform_stamped = self.tf_buffer.lookup_transform("sgr532/ar_tag_link","sgr532/base_link", Time())
            
            # 1. 提取平移向量
            translation = [
                arm_transform_stamped.transform.translation.x,
                arm_transform_stamped.transform.translation.y,
                arm_transform_stamped.transform.translation.z
            ]

            # 2. 提取四元数
            quaternion = [
                arm_transform_stamped.transform.rotation.x,
                arm_transform_stamped.transform.rotation.y,
                arm_transform_stamped.transform.rotation.z,
                arm_transform_stamped.transform.rotation.w
            ]

            # 3. 通过 scipy.spatial.transform.Rotation 转换为旋转矩阵
            rotation_matrix = R.from_quat(quaternion).as_matrix()

            # 4. 输出结果
            # self.get_logger().info(f"🎯 平移向量为: {translation}")
            # self.get_logger().info(f"🎯 旋转矩阵为: {rotation_matrix}")
            # self.get_logger().info(f"🚀 转换信息为: {self.transform_stamped}")
            return True, translation , rotation_matrix

        except TransformException as ex:
            self.get_logger().info(f'\033[33m ❌ Could not transform camera_link to object3: {ex}\033[0m')
            return False, None ,None
        
    
    # 图像回调函数
    def image_callback(self, msg):
        # self.get_logger().info("Image received")
        self.cv_image = self.bridge .imgmsg_to_cv2(msg, desired_encoding='bgr8') # 将ros2图像格式转化成opencv格式

        cv2.imshow("Apriltag", self.cv_image)
        c = cv2.waitKey(1)
        # 检测窗口中是否有按键按下
        # 如果按键“s”按下
        if c == 115:  
            # 监听TF转换信息
            flag, translation, rotation_matrix = self.get_obj_trans()
            arm_flag, arm_translation, arm_rotation_matrix = self.get_arm_trans()
            if flag and arm_flag:
                # 记录当前按下的次数
                self.mark_timer += 1
                # 保存tag的平移向量
                self.t_target2cam.append(translation)
                # 保存tag的旋转矩阵
                self.R_target2cam.append(rotation_matrix)
                # 保存机械臂平移向量
                self.t_base2gripper.append(arm_translation)
                # 保存机械臂旋转矩阵
                self.R_base2gripper.append(arm_rotation_matrix)
                # 输出当前保存的平移向量和旋转矩阵
                self.get_logger().info(f"✅ 已保存第{self.mark_timer}次平移向量和旋转矩阵")
                self.get_logger().info(f"🎯 tag平移向量为: {self.t_target2cam}")
                self.get_logger().info(f"🎯 tag旋转矩阵为: \n{self.R_target2cam}")
                self.get_logger().info(f"🚀 机械臂平移向量为: {self.t_base2gripper}")
                self.get_logger().info(f"🚀 机械臂旋转矩阵为: \n{self.R_base2gripper}")
                self.get_logger().info(f"🎯 共保存{self.mark_timer}次")

        # # 如果按键“c”按下
        elif c == 99:
            # 执行手眼标定
            try:

                # 在调用 `cv2.calibrateHandEye` 之前进行转换
                R_base2gripper = np.array(self.R_base2gripper, dtype=np.float64).reshape(-1, 3, 3)
                t_base2gripper = np.array(self.t_base2gripper, dtype=np.float64).reshape(-1, 3, 1)
                R_target2cam = np.array(self.R_target2cam, dtype=np.float64).reshape(-1, 3, 3)
                t_target2cam = np.array(self.t_target2cam, dtype=np.float64).reshape(-1, 3, 1)

                rmat, pos = cv2.calibrateHandEye(
                    R_gripper2base=R_base2gripper,
                    t_gripper2base=t_base2gripper,
                    R_target2cam=R_target2cam,
                    t_target2cam=t_target2cam,
                    method=4,
                )
                # 将旋转变量转换为欧拉角
                euler_angles = tf_transformations.euler_from_matrix(rmat)

                # 输出结果
                self.get_logger().info(f"🎨 欧拉角: {euler_angles}")
                self.get_logger().info(f"🎨 R_target2cam: {rmat}")
                self.get_logger().info(f"🎨 t_target2cam: {pos}")




                # 归零
                self.R_target2cam = []
                self.t_target2cam = []
                self.R_base2gripper = []
                self.t_base2gripper = []
                
            except cv2.error as e:
                self.get_logger().info(f"❌ 手眼标定失败: {e}")
          

    def delay(self, duration):
        # 非阻塞延迟函数
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('----------over-----------')


def main():
    rclpy.init()
    node = Cam2Object()
    # 使用 MultiThreadedExecutor 执行
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()