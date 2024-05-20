import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # 서비스 메시지 유형
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

# YOLOv5 모델 불러오기
def load_yolo_model():
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    return model

# 이미지를 YOLO 모델로 처리 및 사람 검출
def detect_people(model, img):
    results = model(img)
    detected_objects = results.pandas().xyxy[0]
    people_detected = detected_objects[detected_objects['name'] == 'person']
    return len(people_detected) > 0

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # 로봇에게 명령을 보내기 위한 서비스 클라이언트
        self.cmd_client = self.create_client(SetBool, 'robot_command')
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for command service...')

        # 카메라 이미지를 구독하기 위한 구독자
        self.camera_subscriber = self.create_subscription(
            Image,
            'camera_topic',  # 로봇의 카메라 토픽 이름
            self.camera_callback,
            10)

        # YOLO 모델 불러오기
        self.model = load_yolo_model()

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        self.get_logger().info('Robot Node started')

    def send_command(self, command: bool):
        req = SetBool.Request()
        req.data = command
        self.future = self.cmd_client.call_async(req)
        self.future.add_done_callback(self.command_response_callback)

    def command_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Command response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def camera_callback(self, msg):
        self.get_logger().info('Received camera image')
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지를 YOLO로 처리
        if detect_people(self.model, cv_img):
            self.get_logger().info('Person detected! Sending command.')
            self.send_command(True)
        else:
            self.get_logger().info('No person detected.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()