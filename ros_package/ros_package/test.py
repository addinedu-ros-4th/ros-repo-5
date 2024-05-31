import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('Camera')

        # 웹캠에서 영상을 읽기 위한 VideoCapture 객체 생성
        self.cap = cv2.VideoCapture(0)

        # /camera 토픽으로 영상을 보내기 위한 퍼블리셔 생성
        self.publisher = self.create_publisher(Image, '/camera', 10)

        # OpenCV 이미지와 ROS 이미지 간 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()

        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        # 매 프레임마다 영상을 읽고 /camera 토픽으로 보내기
        self.timer = self.create_timer(0.1, self.send_camera_image)

    def send_camera_image(self):
        ret, frame = self.cap.read()
        if ret:
            # YOLOv5로 객체 감지
            results = self.model(frame)

            # 결과를 프레임에 그리기
            frame = self.draw_results(frame, results)

            # OpenCV 이미지를 ROS 이미지 메시지로 변환
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

            # /camera 토픽으로 영상 메시지 발행
            self.publisher.publish(ros_image)

            # OpenCV 창에 결과 이미지 표시
            cv2.imshow('YOLOv5 Detection', frame)
            cv2.waitKey(1)

    def draw_results(self, frame, results):
        # 결과에서 바운딩 박스, 라벨 및 스코어를 가져와서 이미지에 그리기
        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            label = f'{self.model.names[int(cls)]} {conf:.2f}'
            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
            cv2.putText(frame, label, (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return frame


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
