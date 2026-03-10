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
import time
from geometry_msgs.msg import PoseStamped, Twist

# moveit python library
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


class RobotMoveitAction(Node):
    def __init__(self):
        # 调用父类的初始化方法，设置节点名称为'RobotMoveitAction'
        super().__init__('RobotMoveitAction')

        # 创建 MoveItPy 实例，用于机械臂的运动规划
        self.sgr532 = MoveItPy() 

        # 获取 Aubo 机械臂的运动规划组件
        self.sgr532_arm = self.sgr532.get_planning_component("sagittarius_arm") 

        # 获取机器人模型
        self.robot_model = self.sgr532.get_robot_model()

        # 创建机器人状态实例，用于存储和操作机器人的关节位置等信息
        self.robot_state = RobotState(self.robot_model)

        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("moveit_py.ik_planning") 

        # home状态
        self.home_values = {"joint1": 0.0,"joint2": -0.013962634015954637,"joint3": -0.02268928027592628,"joint4": 0.0,"joint5": -0.013962634015954637,"joint6": 0.0017453292519943296,}
        # up状态
        self.up_values = {"joint1": 0.0,"joint2": -0.0017453292519943296,"joint3": 1.567305668290908,"joint4": 0.0,"joint5": 0.0017453292519943296,"joint6": 0.0017453292519943296,}
        
        # 创建一个 PoseStamped 消息对象
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "sgr532/base_link"

        self.task()

    def ik_motion(self):

        # 设置目标位置的平移分量
        trans_x = 0.228  # Translation: x
        trans_y = 0.000  # Translation: y
        trans_z = 0.105  # Translation: z  
        # 设置目标位置的旋转分量（四元数）
        rot_x = -0.003  # Quaternion x
        rot_y = 0.708   # Quaternion y
        rot_z = -0.002   # Quaternion z
        rot_w = 0.706   # Quaternion w 

        # 设置目标姿态的方向（四元数）
        self.pose_goal.pose.orientation.x = rot_x
        self.pose_goal.pose.orientation.y = rot_y
        self.pose_goal.pose.orientation.z = rot_z
        self.pose_goal.pose.orientation.w = rot_w
        # 设置目标姿态的位置（笛卡尔坐标）
        self.pose_goal.pose.position.x = trans_x
        self.pose_goal.pose.position.y = trans_y
        self.pose_goal.pose.position.z = trans_z
        # 设置机械臂的目标状态，包括姿态和位置
        self.sgr532_arm.set_goal_state(pose_stamped_msg=self.pose_goal, pose_link="sgr532/link_grasping_frame")
        # 将机械臂的起始状态设置为当前状态
        self.sgr532_arm.set_start_state_to_current_state()
        # 进行运动规划
        plan_result = self.sgr532_arm.plan()

        # 执行规划结果
        if plan_result:
            # 如果规划成功，则执行规划
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.sgr532.execute(robot_trajectory, controllers=[])
            self.logger.info("Planning Success !!")
            return True
        else:
            # 如果规划失败，则记录错误日志
            self.logger.error("Planning failed !!")
            return False



    def fk_motion(self,goal):
        """
        正运动学规划

        Returns:
            bool: 如果规划和执行成功，则返回True，否则返回False。
        """

        # 将机械臂的起始状态设置为当前状态
        self.sgr532_arm.set_start_state_to_current_state()

        # 将目标状态设置为预设的“up”状态
        self.robot_state.joint_positions = goal

        # 构造关节约束，确保机械臂的关节位置符合目标状态
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.sgr532.get_robot_model().get_joint_model_group("sagittarius_arm"),
        )

        # 设置机械臂的目标状态，包括关节约束
        self.sgr532_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # 记录日志：机械臂正解运动开始
        self.logger.info("============机械臂正解运动=============")

        # 进行运动规划
        plan_result = self.sgr532_arm.plan()

        # 执行规划结果
        if plan_result:
            # 如果规划成功，则执行规划
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.sgr532.execute(robot_trajectory, controllers=[])
            return True
        else:
            # 如果规划失败，则记录错误日志
            self.logger.error("Planning failed")
            return False
        
    def task(self):
        while True:
            self.fk_motion(self.home_values)
            time.sleep(1)
            self.ik_motion()
            time.sleep(1)
            self.fk_motion(self.up_values )
            time.sleep(1)
            
       
def main():
    rclpy.init()
    node = RobotMoveitAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()