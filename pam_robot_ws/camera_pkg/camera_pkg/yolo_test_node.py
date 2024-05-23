import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # 서비스 메시지 유형
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from msg_pkg.msg import SignalMsg

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
            'camera',  # 로봇의 카메라 토픽 이름
            self.camera_callback,
            10)

        # YOLO 모델 불러오기
        self.model = load_yolo_model()

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        self.get_logger().info('Robot Node started')
        
        self.person_detected = False

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.publisher = self.create_publisher(SignalMsg, 'signal_topic', 10)

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
        self.person_detected = detect_people(self.model, cv_img)

    def timer_callback(self):
        req = SignalMsg()
        if self.person_detected:
            self.get_logger().info('Person detected! Sending command.')
            self.send_command(True)
            req.recv_msg = "Person detected!"
        else:
            self.get_logger().info('No person detected.')
            req.recv_msg = "No person detected."
        
        # 타이머 콜백에서 메시지 발행
        self.publisher.publish(req)
        
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