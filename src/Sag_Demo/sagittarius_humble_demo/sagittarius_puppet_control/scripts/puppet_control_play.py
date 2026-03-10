#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sagittarius_common_msgs.msg import ArmRadControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class RosbagToArmControl(Node):
    def __init__(self):
        super().__init__('rosbag_to_arm_control')

        # Subscriber to the rosbag topic
        self.subscription = self.create_subscription(
            ArmRadControl,
            '/sgr532/joint/commands',
            self.command_callback,
            10
        )

        # Publisher to the target topic
        self.publisher = self.create_publisher(JointState, '/sgr532/joint_states', 10)

        # Joint names for the target message
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint_gripper_left', 'joint_gripper_right'
        ]

    def command_callback(self, msg: ArmRadControl):
        """Callback to handle incoming ArmRadControl messages."""
        # Create a JointState message
        joint_state_msg = JointState()

        # Fill the header
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ''

        # Map rad values to joint positions, and add default values for gripper joints
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = list(msg.rad) + [0.0, 0.0]  # Add two gripper joint positions

        # No velocity or effort data available
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        # Publish the transformed message
        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published JointState: {joint_state_msg}')

def main(args=None):
    rclpy.init(args=args)
    
    node = RosbagToArmControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

