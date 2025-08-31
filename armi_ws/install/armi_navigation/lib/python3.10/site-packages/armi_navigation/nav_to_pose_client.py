import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_queue = []  # 목적지 리스트 (순차 실행용)


    def load_pose(self, waypoint_name):
        pkg_path = get_package_share_directory('armi_navigation')  # 패키지 이름
        yaml_file = os.path.join(pkg_path, 'config', 'waypoints.yaml')

        with open(yaml_file, "r") as f:
            data = yaml.safe_load(f)
        wp = data['waypoints'][waypoint_name]

        pose = PoseStamped()
        pose.header.frame_id = wp['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['position']['x']
        pose.pose.position.y = wp['position']['y']
        pose.pose.position.z = wp['position']['z']
        pose.pose.orientation.x = wp['orientation']['x']
        pose.pose.orientation.y = wp['orientation']['y']
        pose.pose.orientation.z = wp['orientation']['z']
        pose.pose.orientation.w = wp['orientation']['w']
        return pose

    def send_goal(self, waypoint_name):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.load_pose(waypoint_name)

        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: {waypoint_name}")
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal finished!")

        # 다음 목적지가 있으면 실행
        if self.goal_queue:
            next_goal = self.goal_queue.pop(0)
            self.send_goal(next_goal)
        else:
            self.get_logger().info("All goals finished.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseClient()

    # 순차 목적지 등록: 엘베 앞 -> custom_goal
    node.goal_queue = ["custom_goal"]
    node.send_goal("elevator_front")

    rclpy.spin(node)

if __name__ == '__main__':
    main()
