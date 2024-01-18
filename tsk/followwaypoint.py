import numpy
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.navigator = BasicNavigator()

        # Define the waypoints (replace these with your specific waypoints)
        self.waypoints = [
            self.create_pose(0.0, 0.0, 0.0),  # Example Pose 1
            self.create_pose(3.5, 1.0, 1.57),  # Example Pose 2
            self.create_pose(2.0, 2.5, 3.14),  # Example Pose 3
            self.create_pose(0.5, 1.0, 0.0)   # Example Pose 4
        ]

        self.current_waypoint_index = 0

    def create_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = numpy.sin(theta / 2.0)
        pose.pose.orientation.w = numpy.cos(theta / 2.0)
        return pose

    def follow_waypoints(self):
        while rclpy.ok():
            current_pose = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}')
            self.navigator.go_to_pose(current_pose)

            result = self.navigator.wait_for_result()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Arrived at waypoint {self.current_waypoint_index + 1}')
            else:
                self.get_logger().warn(f'Failed to reach waypoint {self.current_waypoint_index + 1}')

            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    waypoint_follower.follow_waypoints()
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()