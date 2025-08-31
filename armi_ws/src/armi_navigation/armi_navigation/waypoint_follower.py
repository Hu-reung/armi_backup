# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import FollowWaypoints
# from rclpy.action import ActionClient
# import yaml
# import os
# from ament_index_python.packages import get_package_share_directory

# class WaypointFollower(Node):
#     def __init__(self):
#         super().__init__('waypoint_follower')
#         self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
#         # 패키지명 변경 반영
#         pkg_share = get_package_share_directory('armi_navigation')
#         yaml_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

#         with open(yaml_file, 'r') as f:
#             self.params = yaml.safe_load(f)

#         self.waypoints = []
#         for wp in self.params['waypoints']:
#             pose = PoseStamped()
#             pose.header.frame_id = wp['frame_id']
#             pose.pose.position.x = wp['position']['x']
#             pose.pose.position.y = wp['position']['y']
#             pose.pose.position.z = wp['position']['z']
#             pose.pose.orientation.x = wp['orientation']['x']
#             pose.pose.orientation.y = wp['orientation']['y']
#             pose.pose.orientation.z = wp['orientation']['z']
#             pose.pose.orientation.w = wp['orientation']['w']
#             self.waypoints.append(pose)

#     def send_goal(self):
#         self._client.wait_for_server()
#         goal_msg = FollowWaypoints.Goal()
#         goal_msg.poses = self.waypoints

#         self.get_logger().info("Sending waypoints...")
#         send_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         send_future.add_done_callback(self.goazl_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             return
#         self.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback)

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f"Current waypoint index: {feedback_msg.feedback.current_waypoint}")

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f"Finished. Missed waypoints: {result.missed_waypoints}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointFollower()
#     node.send_goal()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
