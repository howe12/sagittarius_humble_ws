#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""
import math
import os
import numpy as np

from geometry_msgs.msg import Twist,PoseStamped,TransformStamped, Quaternion, Vector3
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_matrix, rotation_matrix, quaternion_from_matrix
import tf_transformations as tf

import time

from aubo_msgs.srv import AddSignal, SetOutputSignal
from scipy.spatial.transform import Rotation

# moveit python library
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

from sensor_msgs.msg import JointState


class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.joint_order = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        self.joint_positions = [0] * len(self.joint_order)  # 初始化数组
        self.data_received = False

    def joint_state_callback(self, msg):
        # 创建字典以便通过关节名称查找位置
        joint_position_dict = dict(zip(msg.name, msg.position))
        # 按照预定义顺序提取关节位置
        for i, joint_name in enumerate(self.joint_order):
            if joint_name in joint_position_dict:
                self.joint_positions[i] = joint_position_dict[joint_name]
            else:
                self.get_logger().warn(f'Joint name {joint_name} not found in JointState message')
        
        self.get_logger().info(f'Current joint positions: {self.joint_positions}')
        self.data_received = True  # 数据接收完毕，设置标志
        self.subscription.destroy()  # 销毁订阅器以停止接收消息

    def get_current_joint_states(self):
        # 临时订阅当前机械臂关节信息
        self.subscription = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.subscription  # 防止回收未使用变量

        # 等待回调函数获取到数据（简单的等待机制）
        timeout = 5  # 设置超时时间为5秒
        while not self.data_received and timeout > 0 :# 数据接收完毕，设置标志 and timeout > 0:
            rclpy.spin_once(self, timeout_sec=1.0)
            timeout -= 1

        return self.joint_positions

class LLM_motion(Node):
    def __init__(self):
        super().__init__('llm_motion')
        self.logger = get_logger("llm_moveit")

        # 创建 MoveItPy 实例
        self.aubo = MoveItPy()
        # 创建 JointStateListener 实例
        self.joint_state_listener = JointStateListener()

        # 获取 Aubo 机械臂的运动规划组件
        self.aubo_arm = self.aubo.get_planning_component("aubo_arm")
        self.logger.warn("MoveItPy instance created")

        self.robot_model = self.aubo.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        
        # 创建TF坐标监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        # 创建TF坐标变换广播
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        base_link="aubo_base"   
        object_link="tool0"        
        self.transform_stamped = None
        self.joint_timer = self.create_timer(1, lambda:self.get_trans(base_link,object_link))


        self.run_task()

    # 坐标转换
    def get_trans(self,base_link="aubo_base",object_link="tool0"):
        
        try:
            # 获取变换
            self.transform_stamped = self.tf_buffer.lookup_transform(base_link, object_link, Time() )
            # self.get_logger().info(f"****** Transform Stamped: {self.transform_stamped} ******")
            return True, self.transform_stamped
        except Exception as e:
            self.get_logger().info(f"{base_link} to {object_link} transform is not available!")
            return False, None

    # 机械臂运动规划与执行
    def plan_and_execute(self,robot,planning_component,logger,
                         single_plan_parameters=None,
                         multi_plan_parameters=None,
                         sleep_time=0.0,):
        '''
        机械臂规划与执行
        @return: True 为成功到达, False 为失败
        '''
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("成功规划")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            logger.error("规划失败")

        time.sleep(sleep_time)

    # 控制单关节移动方法
    def forward_joint_movement(self,joint_id,angle,clockwise="True"):

        # 角度转弧度
        pi = 3.1415826
        radian = (angle*pi)/180
        
        # 判断是否为顺时针旋转
        if clockwise == "True":
            self.logger.info("---------------------顺时针运动---------------")
            radian = -1*radian
        else:
            self.logger.info("---------------------逆时针运动---------------")
        #  初始化关节目标位置数组
        target_positions = [0]*6 

        # 获取当前关节状态
        current_joint_positions = self.joint_state_listener.get_current_joint_states()
        self.logger.info(f'Current joint positions: {current_joint_positions}')
        self.joint_state_listener.destroy_node()

        # # 赋取当前各个关节角度值
        # for i in current_joint_positions:
        #     target_positions[i] = current_joint_positions[i]

        # 赋取当前各个关节角度值
        for i, position in enumerate(current_joint_positions):
            target_positions[i] = position

        # 更新指定关节角度值
        target_positions[joint_id-1] +=  radian

        # set plan start state to current state
        self.aubo_arm.set_start_state_to_current_state()

        # set constraints message
        joint_values = {
            "shoulder_joint": target_positions[0],
            "upperArm_joint": target_positions[1],
            "foreArm_joint": target_positions[2],
            "wrist1_joint": target_positions[3],
            "wrist2_joint": target_positions[4],
            "wrist3_joint": target_positions[5],
        }
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
                self.robot_state,
                joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
        )
        self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger)

    # 特定位姿移动
    def specific_posture_movement(self,posture_name="zero"):

        # 零位
        self.zero = {
            "foreArm_joint": 0.0,"wrist1_joint": 0.0,"upperArm_joint": 0.0,"wrist2_joint": 0.0,"wrist3_joint": 0.0,"shoulder_joint": 0.0,
        }
        # home
        self.home = {
            "foreArm_joint": -1.541861979076825,"wrist1_joint": -3.6157999327406294e-05,"upperArm_joint": -9.690248793922365e-05,"wrist2_joint": -1.600751241570525,"wrist3_joint": -2.6292469538748267e-05,"shoulder_joint": -9.398494474589825e-05,
        }
        # 在车上的初始姿态
        self.ready_in_car = {
            "foreArm_joint": -2.4108334358394976,"wrist1_joint": 0.19090251921361948,"upperArm_joint": -1.034534295735007,"wrist2_joint": -1.5654776861404676,"wrist3_joint": -0.10093312750890894,"shoulder_joint": 1.5002197993246449,
        }
        # 抓取姿态
        self.ready_values = {
            "foreArm_joint": -0.9656377251059474,"wrist1_joint": 0.26844829995519176,"upperArm_joint": 0.33847095519177534,"wrist2_joint": -1.5632255169254201,"wrist3_joint": -0.30126247877899875,"shoulder_joint": 1.2959143056618647,
        }
        self.ready2_values = {
            "foreArm_joint": -0.2885711017691204,"wrist1_joint": 0.7373690042739373,"upperArm_joint": 0.5566415948364458,"wrist2_joint": -1.5620737561216333,"wrist3_joint": -0.3733832460530537,"shoulder_joint": 1.2129104992004374,
        }
        self.ready_left = {
            "foreArm_joint": -1.2078889688170489,"wrist1_joint": 0.3579811963234353,"upperArm_joint": -0.013439654729536009,"wrist2_joint": -1.5887366519264905,"wrist3_joint": 0.053964028488246114,"shoulder_joint": 1.5866238677768694,
        }
        self.ready_right = {
            "foreArm_joint": -1.0936519036159928,"wrist1_joint": 0.3630834233236494,"upperArm_joint": 0.0958198964245822,"wrist2_joint": -1.5684707970190979,"wrist3_joint": -0.7777613290232959,"shoulder_joint": 0.7547371170316755,
        }
        # 抓取后（前）姿态
        self.catch_values = {
            "foreArm_joint": -1.9574101499793466,"wrist1_joint": 0.2739686821516851,"upperArm_joint": -0.6628273384951079,"wrist2_joint": -1.55778949937379,"wrist3_joint": -0.05559996899298767,"shoulder_joint": 1.5361444659242836,
        }
        # 放置物体姿态
        self.place_values = {
            "foreArm_joint": -0.965094856956392,"wrist1_joint": -0.35666070622992196,"upperArm_joint": 0.9411279617591244,"wrist2_joint": -1.624441236843877,"wrist3_joint": -0.18545916560591436,"shoulder_joint": 1.3522111999819844,
        }
        self.place2_values = {
            "foreArm_joint": -0.9197213501320577,"wrist1_joint": -0.045931047086039824,"upperArm_joint": 0.7063044748240355,"wrist2_joint": -1.5742185969539186,"wrist3_joint": -0.20890886884988916,"shoulder_joint": 1.3430447979161793,
        }

        self.posture = {
            "zero": self.zero,
            "home": self.home,
            "ready_in_car": self.ready_in_car,
            "ready": self.ready_values,
            "ready2": self.ready2_values,
            "ready_left": self.ready_left,
            "ready_right": self.ready_right,
            "catch": self.catch_values,
            "place": self.place_values,
            "place2": self.place2_values,
        }

        # 根据传入的期望位姿设定各个关节的角度值
        target_positions = self.posture[posture_name]

        # 将目标关节位置数组设置为当前机器人状态的关节位置
        # 这一步设置了机器人希望达到的目标关节位置
        self.robot_state.joint_positions = target_positions

        # 构造一个关节约束
        # 该约束基于当前的机器人状态和关节模型组（"aubo_arm" 是机械臂的关节组）
        # 这里使用 get_robot_model().get_joint_model_group("aubo_arm") 获取关节组模型
        joint_constraint = construct_joint_constraint(
            self.robot_state,
            joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
        )

        # 将构造的关节约束设置为运动规划的目标状态
        # motion_plan_constraints 参数是一个列表，包含一个或多个运动规划的约束条件
        self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger)

    # 转换 transform_stamped 为 pose_stamped 格式
    def transform_stamped_to_pose_stamped(self,transform_stamped):
        # 创建一个空的 PoseStamped 消息
        pose_stamped = PoseStamped()

        # 复制 header
        pose_stamped.header = transform_stamped.header

        # 将 translation 和 rotation 从 TransformStamped 复制到 PoseStamped 的 pose 中
        pose_stamped.pose.position.x = transform_stamped.transform.translation.x
        pose_stamped.pose.position.y = transform_stamped.transform.translation.y
        pose_stamped.pose.position.z = transform_stamped.transform.translation.z

        pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x
        pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y
        pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z
        pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w

        return pose_stamped

    def transform_stamped_to_matrix(self,transform_stamped):
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation
        
        # 将四元数转换为旋转矩阵
        rotation_matrix = tf.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        
        # 将平移部分添加到齐次矩阵
        translation_matrix = tf.translation_matrix([translation.x, translation.y, translation.z])
        
        # 将旋转和平移部分组合成一个 4x4 齐次矩阵
        transform_matrix = tf.concatenate_matrices(translation_matrix, rotation_matrix)
        
        return transform_matrix

    def matrix_to_transform_stamped(self,matrix, frame_id, child_frame_id):
        transform_stamped = TransformStamped()
        
        # 提取平移和旋转
        translation = tf.translation_from_matrix(matrix)
        rotation = tf.quaternion_from_matrix(matrix)
        
        # 填充 TransformStamped
        transform_stamped.header.frame_id = frame_id
        transform_stamped.child_frame_id = child_frame_id
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]
        
        return transform_stamped

    def get_translation_from_transform(self,transform_stamped):
        translation = transform_stamped.transform.translation
        return np.array([translation.x, translation.y, translation.z])


    # 逆运动学控制机械臂
    def adjust_position_and_orientation(self,translation_delta=None, rotation_delta=None):

        # 设置机械臂的起始位置为当前状态
        self.aubo_arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "aubo_base"

        # 获取 aubo_base与wrist2_Link 之间的转换关系
        buff, tf_trans = self.get_trans(object_link="wrist2_Link")
        self.get_logger().info(f"****** Transform Stamped: {tf_trans} ******")
        transform_stamped_Wrist2 = tf_trans

        # 获取 wrist2_Link与tool 之间的转换关系
        buff, tf_trans = self.get_trans(base_link="tool0",object_link="wrist2_Link")
        self.get_logger().info(f"****** Transform Stamped: {tf_trans} ******")
        transform_stamped_Wrist3 = tf_trans


        # 提取aubo_base与wrist2_Link位置和四元数
        Wrist2_translation = transform_stamped_Wrist2.transform.translation
        Wrist2_quaternion = [
            transform_stamped_Wrist2 .transform.rotation.x,
            transform_stamped_Wrist2 .transform.rotation.y,
            transform_stamped_Wrist2 .transform.rotation.z,
            transform_stamped_Wrist2 .transform.rotation.w
        ]
        self.get_logger().info(f'----------Wrist2_translation-----------: {Wrist2_translation}')


        # # 提取Wrist3_Link与aubo_base tf位置和四元数
        # Wrist3_translation = transform_stamped_Wrist3.transform.translation
        # Wrist3_quaternion = [
        #     transform_stamped_Wrist3.transform.rotation.x,
        #     transform_stamped_Wrist3.transform.rotation.y,
        #     transform_stamped_Wrist3.transform.rotation.z,
        #     transform_stamped_Wrist3.transform.rotation.w
        # ]


        # 将四元数转换为旋转矩阵
        rotation_mat_Wrist2 = quaternion_matrix(Wrist2_quaternion)[:3, :3]
        # rotation_mat_Wrist3 = quaternion_matrix(Wrist3_quaternion)[:3, :3]


        # 如果有位置增量，则调整位置
        if translation_delta:
            Wrist2_translation.x += translation_delta.get('x', 0.0)
            Wrist2_translation.y += translation_delta.get('y', 0.0)
            Wrist2_translation.z += translation_delta.get('z', 0.0)

        # 如果有旋转增量，则计算新的旋转矩阵
        if rotation_delta:
            for axis, angle in rotation_delta.items():
                angle_rad = np.deg2rad(angle)
                if axis == 'x':
                    rot_mat_delta = rotation_matrix(angle_rad, [1, 0, 0])[:3, :3]
                elif axis == 'y':
                    rot_mat_delta = rotation_matrix(angle_rad, [0, 1, 0])[:3, :3]
                elif axis == 'z':
                    rot_mat_delta = rotation_matrix(angle_rad, [0, 0, 1])[:3, :3]
                
                rotation_mat = np.dot(rotation_mat_Wrist2, rot_mat_delta)
            
            # 90 度对应的弧度值
            angle_rad = np.pi / 2
            # 计算绕 x 轴旋转 90 度的旋转矩阵
            rot_mat_delta = rotation_matrix(angle_rad, [1, 0, 0])[:3, :3]
            rotation_mat = np.dot(rotation_mat_Wrist2, rot_mat_delta)

            # 将Wrist3相对于aubo_base的tf转换与tool0相对于Wrist3的tf进行点积处理
            # rotation_mat = np.dot(rotation_mat_Wrist3,rotation_mat)

        # 将最终的旋转矩阵转换回四元数
        # 创建一个4x4的单位矩阵
        new_transform_matrix = np.identity(4)
        
        # 将新的旋转矩阵填入单位矩阵
        new_transform_matrix[:3, :3] = rotation_mat
        
        # 将旋转矩阵转换为四元数
        new_quaternion = quaternion_from_matrix(new_transform_matrix)
        
        # 获取 Wrist3 的平移部分
        Wrist3_translation = transform_stamped_Wrist3.transform.translation
        self.get_logger().info(f'----------Wrist3_translation-----------: {Wrist3_translation}')

        # 计算新的平移向量
        new_translation = Vector3(
            x=Wrist2_translation.x ,
            y=Wrist2_translation.y ,
            z=Wrist2_translation.z - 0.25400
        )
        # new_translation = Vector3(
        #     x=Wrist2_translation.x + Wrist3_translation.x,
        #     y=Wrist2_translation.y + Wrist3_translation.y,
        #     z=Wrist2_translation.z + Wrist3_translation.z
        # )

        # 更新 TransformStamped 的 translation 和 rotation
        transform_stamped_Wrist2.transform.translation = new_translation
        transform_stamped_Wrist2.transform.rotation = Quaternion(
            x=new_quaternion[0], 
            y=new_quaternion[1], 
            z=new_quaternion[2], 
            w=new_quaternion[3]
        )


        # # 将 TransformStamped 转换为矩阵
        # matrix_base_to_wrist2 = self.transform_stamped_to_matrix(transform_stamped_Wrist2)
        # matrix_wrist2_to_tool = self.transform_stamped_to_matrix(transform_stamped_Wrist3)
        # # 计算从 aubo_base 到 tool0 的新的转换关系
        # matrix_base_to_tool = tf.concatenate_matrices(matrix_base_to_wrist2, matrix_wrist2_to_tool)
        # transform_stamped_base_to_tool = self.matrix_to_transform_stamped(matrix_base_to_tool, frame_id="aubo_base", child_frame_id="tool0")



        # # 发布期望位置tf坐标系信息
        # cube_tf = TransformStamped()
        # cube_tf.header.stamp = self.get_clock().now().to_msg()
        # cube_tf.header.frame_id = "wrist2_Link"
        # cube_tf.child_frame_id = "target"
        # cube_tf.transform.translation = transform_stamped_end.transform.translation
        # cube_tf.transform.rotation = transform_stamped_end.transform.rotation
        # self.tf_pub.sendTransform(cube_tf)
        # self.delay(2)

        # # 获取期望位置与aubo_base的转换关系
        # buff, tf_trans = self.get_trans(object_link="target")
        # self.get_logger().info(f"****** Transform Stamped: {tf_trans} ******")
        # transform_stamped = tf_trans
        # self.delay(2)


        pose_stamped_msg = self.transform_stamped_to_pose_stamped(transform_stamped_Wrist2)
        self.aubo_arm.set_goal_state(pose_stamped_msg=pose_stamped_msg, pose_link="tool0")
        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger, sleep_time=5.0)


        return transform_stamped_Wrist2
    
    # 休眠操作
    def delay(self, duration):
        # 非阻塞延迟函数
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('----------over-----------')

    # 主任务程序
    def run_task(self):

        ###################################################################
        # 基础控制方法
        ###################################################################
        # 控制单关节移动方法（第x关节旋转y角度）
        # self.forward_joint_movement(1,60)

        # 特定位姿移动 （移动到零位）
        # self.specific_posture_movement("home")
        self.delay(2)

        # 机械臂逆运动学控制 （位置（向上/下/前/后/左/右）0.1米；朝向（往上看、往下看、往左看、往右看），30度）

        translation_delta = {'x': 0.10,'y': 0.0,'z': 0.0}  # 单位为米，x增加表示向前，y增加表示向右，z增加表示向上
        rotation_delta = {'x': 0.0}  # 单位为角度，右手定则，角度为负逆时针旋转
        new_transform = self.adjust_position_and_orientation(translation_delta, rotation_delta)

def main():
    rclpy.init()
    node = LLM_motion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
     
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
