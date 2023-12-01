#!/usr/bin/env python3

"""
Author: Quy Hoang
Date Started: Nov 22, 2023

This script defines a ROS2 node responsible for handling robot navigation. Upon receiving a destination command via the 'destination' topic, the node computes and navigates the robot to predefined positions. Utilizing the BasicNavigator from the `nav2_simple_commander`, it translates string messages ('a', 'b', 'c', 'o') into corresponding spatial goals.
"""


import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf_transformations

class TransportNode(Node):
    def __init__(self):
        super().__init__('transport')  # Renamed node



        self.subscription = self.create_subscription(String,'destination',self.destination_callback,10)
        self.subscription  # prevent unused variable warning
        self.navigator = BasicNavigator()

        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        
        self.navigator.waitUntilNav2Active()

        # Define goal poses
        self.goals = {
            # 'a': self.create_pose_stamped(3.5, 1.0, 1.57),
            # 'b': self.create_pose_stamped(2.0, 2.5, 3.14),
            # 'c': self.create_pose_stamped(0.5, 1.0, 0.0),
            # 'o': self.create_pose_stamped(0.0, 0.0, 0.0)

            'a': self.create_pose_stamped(1.4, -0.5, 1.57),
            'b': self.create_pose_stamped(1.5, 0.5, 3.14),
            'c': self.create_pose_stamped(0.6, -0.6, 4.7),
            'o': self.create_pose_stamped(0.0, 0.0, 0.0)
        }

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def destination_callback(self, msg):
        destination = msg.data
        if destination in self.goals:
            goal = self.goals[destination]
            self.get_logger().info(f"Going to destination {destination}")
            self.navigator.goToPose(goal)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # Optionally print feedback
                # print(feedback)
            result = self.navigator.getResult()
            self.get_logger().info(f"Arrived at destination {destination} with result: {result}")

def main(args=None):
    rclpy.init(args=args)
    transport_node = TransportNode()
    try:
        rclpy.spin(transport_node)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        transport_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
