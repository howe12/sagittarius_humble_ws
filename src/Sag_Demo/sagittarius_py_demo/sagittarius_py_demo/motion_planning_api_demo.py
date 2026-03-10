# Copyright (c) 2024 NXROBO
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

import math
import _thread
import threading
import os
import sys 
import pickle

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
import tf_transformations
from rclpy.time import Time

import time
from rclpy.duration import Duration
from std_srvs.srv import SetBool


from turtlesim.srv import Spawn
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
# from aubo_msgs.srv import AddSignal, SetOutputSignal, GetSignalStatus
from scipy.spatial.transform import Rotation

from moveit.core.kinematic_constraints import construct_joint_constraint

import inspect
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


from geometry_msgs.msg import TransformStamped

import numpy as np
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import cv2
from ultralytics import YOLO

# 机械臂运动规划
class RobotMoveitAction(Node):
    def __init__(self):
        super().__init__('RobotMoveitAction')   
        logger = get_logger("moveit_py.pose_goal") # 创建日志记录器 
        try:
            self.sgr532 = MoveItPy()  # 创建 MoveItPy 实例
        except Exception as e:
            print(f"Error initializing MoveItPy: {e}")
            raise
        # self.sgr532 = MoveItPy() # 创建 MoveItPy 实例
        self.sgr532_arm = self.sgr532.get_planning_component("sagittarius_arm") # 获取 Aubo 机械臂的运动规划组件
        self.robot_model = self.sgr532.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # 零位状态
        self.zero_values = {"joint1": 0.0,"joint2": 0.0,"joint3": 0.0,"joint4": 0.0,"joint5": 0.0,"joint6": 0.0,}
        # home状态
        self.home_values = {"joint1": 0.0,"joint2": -0.013962634015954637,"joint3": -0.02268928027592628,"joint4": 0.0,"joint5": -0.013962634015954637,"joint6": 0.0017453292519943296,}
        # up状态
        self.up_values = {"joint1": 0.0,"joint2": -0.0017453292519943296,"joint3": 1.567305668290908,"joint4": 0.0,"joint5": 0.0017453292519943296,"joint6": 0.0017453292519943296,}

        self.posture = {
            "zero": self.zero_values,
            "home": self.home_values,
            "up": self.up_values,
        }

        # 记录日志：MoveItPy 实例已创建
        logger.info("MoveItPy instance created")

        # 创建一个 PoseStamped 消息对象
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "sgr532/base_link"

    # 机械臂运动规划与执行
    def plan_and_execute(self,robot,planning_component,logger,
                         single_plan_parameters=None,
                         multi_plan_parameters=None,sleep_time=0.0,):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            # 如果有多个规划参数，则进行多点规划
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            # 如果有单个规划参数，则进行单点规划
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            # 否则，进行默认规划
            plan_result = planning_component.plan()

        # 执行规划结果
        if plan_result:
            # 如果规划成功，则执行规划
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            return True
        else:
            # 如果规划失败，则记录错误日志
            logger.error("Planning failed")
            return False

        time.sleep(sleep_time)
    

    def determine_pose(self,pose):
        """设定目标姿态,通过正运动学规划控制"""
    
        logger = get_logger("moveit_py.pose_goal")

        # 根据传入的 pose 参数进行判断并设置不同的预设位姿值
        if pose != None:

            self.sgr532_arm.set_start_state_to_current_state()

            self.robot_state.joint_positions = self.posture[pose]
            joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.sgr532.get_robot_model().get_joint_model_group("sagittarius_arm"),
            )
            self.sgr532_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
            logger.info("============机械臂正解运动=============")
            # plan to goal
            self.plan_and_execute(self.sgr532, self.sgr532_arm, logger, sleep_time=3.0)

        else:
            raise ValueError("Invalid pose parameter. Please choose from 'predict_pose', 'catch_pose_front', 'catch_pose_back', or 'place_pose'.")
        return self.pose_goal

                
    # 抓取物体
    def pitch_object(self, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w):

        logger = get_logger("AutoAction.pose_object")

        for i in range(3):
            self.pose_goal.pose.orientation.x = rot_x
            self.pose_goal.pose.orientation.y = rot_y
            self.pose_goal.pose.orientation.z = rot_z
            self.pose_goal.pose.orientation.w = rot_w
            self.pose_goal.pose.position.x = trans_x
            self.pose_goal.pose.position.y = trans_y
            self.pose_goal.pose.position.z = trans_z
            self.sgr532_arm.set_goal_state(pose_stamped_msg=self.pose_goal, pose_link="sgr532/link_grasping_frame")
            self.sgr532_arm.set_start_state_to_current_state()
            ret = self.plan_and_execute(self.sgr532, self.sgr532_arm, logger, sleep_time=3.0)
            if ret != False:
                logger.info("--------------------------------规划成功，机械臂移动----------------------------------------")
                break
            else:
                logger.info("----------------------------------机械臂运动规划失败----------------------------------------")
                return False
        return ret
    
    def ready(self):
        logger = get_logger("################### AutoAction.pose_object ###################")
        # set plan start state using predefined state
        # self.sgr532_arm.set_start_state(configuration_name="zero")
        self.sgr532_arm.set_start_state_to_current_state()
        # set pose goal using predefined state
        self.sgr532_arm.set_goal_state(configuration_name="home")

        # plan to goal
        self.plan_and_execute(self.sgr532, self.sgr532_arm, logger, sleep_time=3.0)


class AutoAction(Node):
    def __init__(self):

        super().__init__('AutoAction')
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
   
        self.moveit = RobotMoveitAction()

        self.transform_stamped = None

        # 创建日志记录器
        self.logger = get_logger("moveit_py.pose_goal") # 创建日志记录器

        # self.logger(inspect.getfile(MoveItPy))
        # self.logger(inspect.getfile(MultiPipelinePlanRequestParameters))
        time.sleep(5)
        # self.moveit.ready()

        self.arm_ready()


    def arm_ready(self):

        # 机械臂正运动学运动规划与执行
        pose = "home"
        self.moveit.determine_pose(pose) # 移动机械臂到初始姿态
        time.sleep(5)
        self.logger.info("机械臂就绪home")

        pose = "up"
        self.moveit.determine_pose(pose) # 移动机械臂到初始姿态
        time.sleep(3)
        self.logger.info("机械臂就绪up")


        # 机械臂逆运动学运动规划与执行
        trans_x = 0.368  # Translation: x
        trans_y = 0.001  # Translation: y
        trans_z = 0.281  # Translation: z  
        rot_x = 0.000  # Quaternion x
        rot_y = 0.045   # Quaternion y
        rot_z = 0.001   # Quaternion z
        rot_w = 0.999   # Quaternion w             
        # ori = tf_transformations.quaternion_from_euler(0, 0, 0)           # 计算姿态的四元数，让机械臂朝向下抓取
        # rot_x = ori[0]
        # rot_y = ori[1]
        # rot_z = ori[2]
        # rot_w = ori[3]

        self.logger.info("\n逆运动学消息:\nTranslation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\nRotation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\n  w:%.6f\n" \
                                % (trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w))
        self.moveit.pitch_object(trans_x, trans_y, trans_z  , rot_x, rot_y, rot_z, rot_w) 

            
def main():
    rclpy.init()
    node = AutoAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()