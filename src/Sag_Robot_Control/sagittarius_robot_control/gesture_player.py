#!/usr/bin/env python3
"""
Sagittarius Robot Gesture Player
用于播放预设的机械臂动作/姿态

使用方法:
  ros2 topic pub /robot_gesture/play std_msgs/msg/String "{data: 'shake_head'}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
import yaml
import os
import time
import threading


class GesturePlayer(Node):
    """机械臂动作播放器"""
    
    def __init__(self):
        super().__init__('gesture_player')
        
        # 加载动作库
        self.gestures = {}
        self.poses = {}
        self.load_gestures()
        
        # ROS2 Action Client
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/sgr532/arm_controller/follow_joint_trajectory'
        )
        
        # 订阅动作命令话题
        self.subscription = self.create_subscription(
            String,
            '/robot_gesture/play',
            self.play_callback,
            10
        )
        
        self.get_logger().info('='*50)
        self.get_logger().info('Gesture Player 已启动!')
        self.get_logger().info(f'可用动作: {list(self.gestures.keys())}')
        self.get_logger().info('调用方式: ros2 topic pub /robot_gesture/play std_msgs/msg/String "{data: 动作名}"')
        self.get_logger().info('='*50)
        
        # 当前播放状态
        self.is_playing = False
        self.current_gesture = None
        
    def load_gestures(self):
        """加载动作库"""
        path = '/home/leo/Music/sagittarius_humble_ws/src/Sag_Robot_Control/config/gestures.yaml'
        
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                self.poses = data.get('poses', {})
                self.gestures = data.get('gestures', {})
                self.get_logger().info(f'已加载动作库: {path}')
        else:
            self.get_logger().error(f'无法找到 gestures.yaml: {path}')
        
    def get_joint_names(self):
        return ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    def resolve_pose(self, pose_name):
        """解析姿态"""
        if pose_name in self.poses:
            pose_data = self.poses[pose_name]
            return pose_data.get('joints', []), pose_data.get('duration', 1.0)
        elif isinstance(pose_name, list):
            return pose_name, 1.0
        else:
            self.get_logger().warn(f'未知姿态: {pose_name}')
            return [0.0] * 6, 1.0
    
    def play_callback(self, msg):
        """接收播放命令"""
        gesture_name = msg.data.strip()
        self.get_logger().info(f'收到播放请求: {gesture_name}')
        
        if gesture_name == 'stop':
            self.stop_playing()
            return
            
        if gesture_name in self.gestures:
            gesture = self.gestures[gesture_name]
            if gesture.get('loop', False):
                self.play_loop(gesture_name)
            else:
                self.play_sequence(gesture_name)
        else:
            # 尝试作为单个姿态播放
            joints, duration = self.resolve_pose(gesture_name)
            if joints != [0.0] * 6 or gesture_name == 'home':
                self.play_single_pose(joints, duration)
            else:
                self.get_logger().error(f'未知动作: {gesture_name}')
    
    def play_single_pose(self, joints, duration):
        """播放单个姿态"""
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server 不可用!')
            return
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.get_joint_names()
        
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        goal_msg.trajectory.points.append(point)
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
    def play_sequence(self, gesture_name):
        """播放动作序列"""
        gesture = self.gestures[gesture_name]
        sequence = gesture.get('sequence', [])
        
        self.get_logger().info(f'播放序列: {gesture_name}')
        
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server 不可用!')
            return
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.get_joint_names()
        
        current_time = 0.0
        for step in sequence:
            pose_name = step.get('pose')
            joints, _ = self.resolve_pose(pose_name)
            duration = step.get('duration', 1.0)
            
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = rclpy.duration.Duration(seconds=current_time + duration).to_msg()
            goal_msg.trajectory.points.append(point)
            current_time += duration
            
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
    def play_loop(self, gesture_name):
        """循环播放"""
        self.is_playing = True
        self.current_gesture = gesture_name
        
        def loop_task():
            while self.is_playing and rclpy.ok():
                self.play_sequence(gesture_name)
                time.sleep(0.2)
                
        thread = threading.Thread(target=loop_task, daemon=True)
        thread.start()
        
    def stop_playing(self):
        """停止播放"""
        self.is_playing = False
        self.get_logger().info('停止播放，归位...')
        joints, _ = self.resolve_pose('home')
        self.play_single_pose(joints, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GesturePlayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
