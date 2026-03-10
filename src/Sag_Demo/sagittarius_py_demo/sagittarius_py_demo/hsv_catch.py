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
from rclpy.callback_groups import ReentrantCallbackGroup


from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
from rclpy.time import Time
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer



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

        self.sgr532_gripper = self.sgr532.get_planning_component("sagittarius_gripper") 
        # 获取机器人模型
        self.robot_model = self.sgr532.get_robot_model()

        # 创建机器人状态实例，用于存储和操作机器人的关节位置等信息
        self.robot_state = RobotState(self.robot_model)

        # 创建日志记录器，用于记录机械臂运动规划的相关信息
        self.logger = get_logger("moveit_py.ik_planning") 

        # 机械臂检测状态
        self.detect_values = {"joint1": 0.0,"joint2": 0.5393067388662478,"joint3": -0.3543018381548489,"joint4": 0.0,"joint5": -1.7924531417981766,"joint6": -0.003490658503988659,}
        # 机械臂放置状态
        self.place_values = {"joint1": -1.5446163880149815,"joint2": 0.5393067388662478,"joint3": -0.3543018381548489,"joint4": 0.0,"joint5": -1.7924531417981766,"joint6": -0.003490658503988659,}
        # 夹爪抓取状态
        self.gripper_close_values = {"joint_gripper_left": -0.028,"joint_gripper_right": -0.028,}
        # 夹爪释放状态
        self.gripper_open_values = {"joint_gripper_left": -0.0,"joint_gripper_right": -0.0,}

        # 创建一个 PoseStamped 消息对象
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "sgr532/base_link"

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        # self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.transform_stamped = None
        self.grasp_flag = False
        self.min_rect = None

        time.sleep(7) # 休眠等待机械臂服务启动
        self.gripper_fk_motion(self.gripper_open_values)
        time.sleep(1) 
        self.fk_motion(self.detect_values)

        self.grasp_timer = self.create_timer(2, self.get_trans)
        # self.grasp_task = self.create_timer(1, self.task)

        self.task()

    def get_trans(self):
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.delay(1)
        # self.logger.info(f'\033[33m 当前抓取状态为: {self.grasp_flag}\033[0m')
        # if self.grasp_flag == False:
        # 获取物体坐标转换
        self.transform_stamped = None
        try:
            self.transform_stamped = self.tf_buffer.lookup_transform("sgr532/base_link", "object",Time())
            self.grasp_flag = True
            # self.get_logger().info(f"转换信息为: {self.transform_stamped}")
            # self.task()
            # self.transform_stamped = None
            return True, self.transform_stamped   

        except TransformException as ex:
            self.get_logger().info(f'\033[33mCould not transform base_link to object: {ex}\033[0m')
            # self.get_logger().warn(f"\033[33mWarning: Transform delayed\033[0m")   
            return False, None 

    def ik_motion(self,trans_x, trans_y, trans_z , rot_x, rot_y, rot_z, rot_w):

        # # 设置目标位置的平移分量
        # trans_x = 0.228  # Translation: x
        # trans_y = 0.000  # Translation: y
        # trans_z = 0.105  # Translation: z  
        # # 设置目标位置的旋转分量（四元数）
        # rot_x = -0.003  # Quaternion x
        # rot_y = 0.708   # Quaternion y
        # rot_z = -0.002   # Quaternion z
        # rot_w = 0.706   # Quaternion w 

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
        
    def gripper_fk_motion(self,goal):
        """
        夹爪正运动学规划
        Returns:
            bool: 如果规划和执行成功，则返回True，否则返回False。
        """
        # 将机械臂的起始状态设置为当前状态
        self.sgr532_gripper.set_start_state_to_current_state()
        self.robot_state.joint_positions = goal
        # 构造关节约束，确保机械臂的关节位置符合目标状态
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.sgr532.get_robot_model().get_joint_model_group("sagittarius_gripper"),
        )
        # 设置机械臂的目标状态，包括关节约束
        self.sgr532_gripper.set_goal_state(motion_plan_constraints=[joint_constraint])

        # 进行运动规划
        plan_result = self.sgr532_gripper.plan()

        # 执行规划结果
        if plan_result:
            # 如果规划成功，则执行规划
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.sgr532.execute(robot_trajectory, controllers=[])
        else:
            # 如果规划失败，则记录错误日志
            self.logger.error("Planning failed")


    def motion(self):
        # while rclpy.ok():
        flag, trans = self.get_trans()

        if flag:

            trans_x = trans.transform.translation.x                    
            trans_y = trans.transform.translation.y                    
            trans_z = trans.transform.translation.z 
            rot_w = trans.transform.rotation.w
            rot_x = trans.transform.rotation.x
            rot_y = trans.transform.rotation.y
            rot_z = trans.transform.rotation.z   
            
            self.logger.info("\033[34m\n物体坐标变换消息:\nTranslation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\nRotation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\n  w:%.6f\n\033[0m" \
                                    % (trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w))
            self.logger.info("========  开始抓取 ===== ")
            self.ik_motion(trans_x, trans_y, trans_z+0.07  , rot_x, rot_y, rot_z, rot_w)  # 机械臂移动到物体上方
            time.sleep(1)
            self.ik_motion(trans_x, trans_y, trans_z+0.04  , rot_x, rot_y, rot_z, rot_w)       # 机械臂向下移动
            time.sleep(1)
            self.gripper_fk_motion(self.gripper_close_values)
            time.sleep(1)
            self.fk_motion(self.place_values)
            time.sleep(1)
            self.gripper_fk_motion(self.gripper_open_values)
            time.sleep(1)
            self.fk_motion(self.detect_values)
            time.sleep(1)


    def delay(self, duration):
        # 非阻塞延迟函数
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('----------over-----------')

    def task(self):
        while True:
            self.delay(1)
            self.motion()
            # self.get_logger().info(f'\033[33m 当前抓取状态为: {self.grasp_flag}\033[0m')
            # time.sleep(1)



            
       
def main():
    rclpy.init()
    node = RobotMoveitAction()
    # 使用 MultiThreadedExecutor 执行
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()