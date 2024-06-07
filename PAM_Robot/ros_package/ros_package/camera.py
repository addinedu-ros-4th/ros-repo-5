import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('Camera')

        # 웹캠에서 영상을 읽기 위한 VideoCapture 객체 생성
        self.cap = cv2.VideoCapture(0)

        # /camera/compressed 토픽으로 영상을 보내기 위한 퍼블리셔 생성
        self.publisher = self.create_publisher(CompressedImage, '/camera/compressed', 10)

        # OpenCV 이미지와 ROS 이미지 간 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()

        # 매 프레임마다 영상을 읽고 /camera/compressed 토픽으로 보내기
        self.timer = self.create_timer(0.1, self.send_camera_image)

    def send_camera_image(self):
        ret, frame = self.cap.read()
        if ret:
            # 이미지 크기를 320x240으로 변경
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.resize(frame, (320, 240))
            # 이미지를 180도 회전
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # OpenCV 이미지를 JPEG 형식으로 압축
            _, compressed_image = cv2.imencode('.jpg', frame)
            
            # 압축된 이미지를 ROS CompressedImage 메시지로 변환
            ros_image = CompressedImage()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.format = "jpeg"
            ros_image.data = compressed_image.tobytes()

            # /camera/compressed 토픽으로 영상 메시지 발행
            self.publisher.publish(ros_image)


def main(args=None):
    rp.init(args=args)

    camera_node = CameraNode()

    rp.spin(camera_node)

    # 프로그램 종료 시 웹캠 연결 해제
    camera_node.cap.release()
    cv2.destroyAllWindows()

    rp.shutdown()

if __name__ == '__main__':
    main()
