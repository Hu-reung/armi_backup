from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS 적용해서 퍼블리셔 생성
        self.publisher = self.create_publisher(Image, 'camera/image_raw', qos_profile)

        # QoS 적용해서 구독자 생성
        self.subscription = self.create_subscription(String, 'yolo/detection_result', self.detection_callback, qos_profile)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
        else:
            self.get_logger().info("카메라 연결 성공")

        self.timer = self.create_timer(0.1, self.publish_frame) # 0.1 -> 1.0으로 수정

    def publish_frame(self):
        ret, frame = self.cap.read() 
        #카메라 캡쳐 (ret : 성공여부, frame : 이미지 데이터_NumPy 배열_3차원배열(높이, 너비, 채널))
        if ret:
		        # OpenCV 이미지를 ROS 이미지 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # ROS 토픽으로 퍼블리시
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("카메라에서 프레임을 읽을 수 없습니다.")

    def detection_callback(self, msg):
        self.get_logger().info(f"탐지 결과 수신: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
