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

class CamHSVDetect(Node):
    def __init__(self):
        # 调用父类的初始化方法，设置节点名称为'RobotMoveitAction'
        super().__init__('CamHSVDetect')

        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("cam_hsv_detect") 

        # 创建Opencv转换器
        # self.bridge = CvBridge()
        # 订阅相机信息话题
        self.sub_info = self.create_subscription(CameraInfo, '/camera1/camera_info', self.imageCameraInfoCallback, 10)
        # 创建图像订阅者
        self.image_subscriber = self.create_subscription(Image,  '/camera1/image_raw', self.image_callback, 10)

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fx = 759.63
        self.fy = 760.08
        self.cx = 303.25
        self.cy = 230.05

        self.readings = []

        self.upload_application_parameters()

        time.sleep(2)

    def upload_application_parameters(self):
        # 设置参数文件路径为当前功能包下的cofig文件夹中的usb_cam_calibration.yaml
        file_path = os.path.join(get_package_share_directory("sagittarius_cam_calibration"), "config", "usb_cam_calibration.yaml")
        if os.path.exists(file_path):
            # read the yaml file
            with open(file_path, "r") as file:
                self.config = yaml.load(file, Loader=yaml.FullLoader)

            # # camera topic
            # if self.config["camera_image_topic"]!="":
            #     self.update_camera_topic(self.config["camera_image_topic"])

            # if self.config["camera_info_topic"]!="":
            #     self.update_camera_info_topic(self.config["camera_info_topic"])

            # aruco board detection
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)  
            self.charuco_board = cv2.aruco.CharucoBoard(
                [self.config["charuco"]["squares_x"],self.config["charuco"]["squares_y"]],
                self.config["charuco"]["square_length"],
                self.config["charuco"]["marker_length"],
                self.aruco_dict
            )
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_FOCAL_LENGTH
            self.cv_bridge = CvBridge()

    def imageCameraInfoCallback(self, msg):
        # 获取图像信息
        self.camera_info = msg
    
    # 发布tf变换
    def publish_tf(self, t_target2cam, R_target2cam):
        # 根据矩阵数量循环发布tf变换
        for i in range(len(t_target2cam)):
            
            # 创建tf变换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "sgr532/usb_cam_link"
            t.child_frame_id = "object"+str(i+1)
            
            # 设置平移向量
            t.transform.translation.x = float(t_target2cam[i][0])
            t.transform.translation.y = float(t_target2cam[i][1])
            t.transform.translation.z = float(t_target2cam[i][2])
            
            # 将旋转矩阵转换为四元数
            rotation_matrix = R_target2cam[i]
            rotation = R.from_matrix(rotation_matrix)  # 从旋转矩阵创建 Rotation 对象
            quat = rotation.as_quat()  # 转换为四元数 [x, y, z, w]

            # 设置四元数
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            # 发布tf变换
            self.tf_pub.sendTransform(t)

        
    # 图像回调函数
    def image_callback(self, msg):
        # self.get_logger().info("Image received")
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 将ros2图像格式转化成opencv格式

        # 检测charuco板
        image, charuco_corners, charuco_ids, image_shape = self.detect_charuco_board(self.cv_image)

        # 显示检测结果
        if charuco_corners is not None:
            cv2.imshow("Charuco Board", image)
            c = cv2.waitKey(1)
            # 检测窗口中是否有按键按下
            # 如果按键“s”按下
            if c == 115:
                # 保存相机内参
                self.readings.append(self.detect_charuco_board(self.cv_image))
                # 输出reading内容，以蓝色显示
                self.logger.info(f"\033[94mReadings:{self.readings}\033[0m")
            # 如果按键“c”按下
            elif c == 99:
                # 提示进行相机标定
                self.logger.info("\033[94m正在计算图像到相机的矩阵\033[0m")
                # 计目标到相机的变换
                R_target2cam, t_target2cam, successes = self.calc_target_to_camera(self.readings)
                # 输出变换矩阵
                self.logger.info(f"\033[92mR_target2cam:{R_target2cam}\033[0m")
                self.logger.info(f"\033[92mt_target2cam:{t_target2cam}\033[0m")
                # 生成tf变换
                self.publish_tf(t_target2cam, R_target2cam)

                self.readings = []
          

    def detect_charuco_board(self, image):
        """
        Detect charuco board in image

        Adapted from: https://github.com/AlexanderKhazatsky/R2D2/blob/1aa471ae35cd9b11e20cc004c15ad4c74e92605d/r2d2/calibration/calibration_utils.py#L122
        """
        # 使用aruco库检测图像中的aruco标记
        # corners: 检测到的标记角点
        # ids: 检测到的标记ID
        # rejectedImgPoints: 被拒绝的标记点（未通过检测的标记）
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            image, 
            self.aruco_dict,  # 使用的aruco字典
            parameters=self.detector_params,  # 检测参数
        )
        
        # 使用refineDetectedMarkers函数优化检测到的标记
        # 该函数可以尝试重新检测被拒绝的标记，并返回优化后的角点和ID
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            image,
            self.charuco_board,  # charuco板对象
            corners,
            ids,
            rejectedImgPoints,
            parameters=self.detector_params,  # 检测参数
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),  # 相机内参矩阵
            distCoeffs=np.array(self.camera_info.d),  # 相机畸变系数
        )

        # 如果没有检测到任何标记，记录日志并返回None
        if ids is None:
            # self.logger.info("No markers found!")
            return None, None,None, None
        # else:
        #     # 如果检测到标记，记录检测到的标记数量
        #     self.logger.info(f"Found {len(ids)} markers!")

        # 使用interpolateCornersCharuco函数检测charuco板的角点
        # num_corners_found: 检测到的charuco角点数量
        # charuco_corners: 检测到的charuco角点坐标
        # charuco_ids: 检测到的charuco角点ID
        num_corners_found, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners,  # 检测到的aruco标记角点
            ids,  # 检测到的aruco标记ID
            image,  # 输入图像
            self.charuco_board,  # charuco板对象
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),  # 相机内参矩阵
            distCoeffs=np.array(self.camera_info.d),  # 相机畸变系数
        )

        # 如果检测到的charuco角点数量少于5个，记录日志并返回None
        if num_corners_found < 5:
            # self.logger.info("Charuco board not found!")
            return None, None,None, None

        # 在图像上绘制检测到的charuco角点
        image = aruco.drawDetectedCornersCharuco(
            image, charuco_corners,
        )
        
        # 返回处理后的图像、charuco角点、charuco角点ID以及图像的尺寸
        return image, charuco_corners, charuco_ids, image.shape[:2]
       
    def calc_target_to_camera(self, readings):
        """
        Calculate target to camera transform

        Adapted from: https://github.com/AlexanderKhazatsky/R2D2/blob/1aa471ae35cd9b11e20cc004c15ad4c74e92605d/r2d2/calibration/calibration_utils.py#L164
        """
        # 初始化变量
        init_corners_all = []  # 存储所有图像中检测到的Charuco角点
        init_ids_all = []  # 存储所有图像中检测到的Charuco角点对应的ID
        fixed_image_size = readings[0][3]  # 获取第一张图像的尺寸，假设所有图像的尺寸相同
        # 输出第一张图像的尺寸
        self.logger.info(f"fixed_image_size:{fixed_image_size}")

        # 处理每一张图像的检测结果
        init_successes = []  # 存储成功处理的图像索引
        for i in range(len(readings)):
            corners, charuco_corners, charuco_ids, img_size = readings[i]  # 解包每一张图像的检测结果
            # assert img_size == fixed_image_size  # 确保所有图像的尺寸一致
            if img_size != fixed_image_size:
                # 调整图像大小
                # image = cv2.resize(readings[i][0], fixed_image_size)
                img_size = fixed_image_size
            init_corners_all.append(charuco_corners)  # 将当前图像的Charuco角点添加到列表中
            init_ids_all.append(charuco_ids)  # 将当前图像的Charuco角点ID添加到列表中
            # 输出corners角点
            self.logger.info(f"init_corners_all:{init_corners_all}")
            # 输出charuco_ids角点
            self.logger.info(f"init_ids_all:{init_ids_all}")
            
            init_successes.append(i)  # 记录当前图像的索引

        # 第一轮：检测异常值
        threshold = 10  # 设置成功处理的图像数量的阈值
        if len(init_successes) < threshold:  # 如果成功处理的图像数量少于阈值
            self.logger.info("len(init_successes) < threshold")
            return None,None,None  # 返回None，表示无法进行校准

        # 使用Charuco角点进行相机校准，计算相机内参、外参和误差
        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors = (
            aruco.calibrateCameraCharucoExtended(
                charucoCorners=init_corners_all,  # 所有图像的Charuco角点
                charucoIds=init_ids_all,  # 所有图像的Charuco角点ID
                board=self.charuco_board,  # Charuco板对象
                imageSize=fixed_image_size,  # 图像尺寸
                flags=self.calib_flags,  # 校准标志
                cameraMatrix=np.array(self.camera_info.k).reshape(3,3),  # 相机内参矩阵
                distCoeffs=np.array(self.camera_info.d),  # 相机畸变系数
            )
        )
        print(f"perViewErrors:{perViewErrors}")  # 打印每张图像的校准误差

        # 移除异常值
        # 只保留误差小于等于3.0的图像数据
        final_corners_all = [
            init_corners_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        final_ids_all = [
            init_ids_all[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        final_successes = [
            init_successes[i] for i in range(len(perViewErrors)) if perViewErrors[i] <= 3.0
        ]
        if len(final_successes) < threshold:  # 如果剩余图像数量少于阈值
            return None,None,None  # 返回None，表示无法进行校准

        # 第二轮：计算最终的外参
        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=final_corners_all,  # 过滤后的Charuco角点
            charucoIds=final_ids_all,  # 过滤后的Charuco角点ID
            board=self.charuco_board,  # Charuco板对象
            imageSize=fixed_image_size,  # 图像尺寸
            flags=self.calib_flags,  # 校准标志
            cameraMatrix=np.array(self.camera_info.k).reshape(3,3),  # 相机内参矩阵
            distCoeffs=np.array(self.camera_info.d),  # 相机畸变系数
        )
        
        # 返回变换结果
        if calibration_error > 3.0:  # 如果校准误差大于3.0
            self.logger.info("calibration_error > 3.0")
            return None,None,None  # 返回None，表示校准失败

        # 将旋转向量（rvecs）转换为旋转矩阵（rmats）
        rmats = [R.from_rotvec(rvec.flatten()).as_matrix() for rvec in rvecs]
        # 将平移向量（tvecs）展平
        tvecs = [tvec.flatten() for tvec in tvecs]

        # 返回旋转矩阵、平移向量和成功处理的图像索引
        return rmats, tvecs, final_successes

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