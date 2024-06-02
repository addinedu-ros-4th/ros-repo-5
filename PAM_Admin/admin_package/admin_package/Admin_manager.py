import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ros_package_msgs.srv import CommandString
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import torch
import numpy as np
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import Voice
from ros_package_msgs.msg import RobotState
from .DB_Manager import DatabaseManager
import time
import cv2
import threading

# YOLOv5 모델 불러오기
def load_yolo_model():
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    return model

class Admin_Manager(Node):
    def __init__(self):
        super().__init__('Admin_Manager')

        # DB_Manager 인스턴스 생성
        self.db_manager = DatabaseManager(host="localhost", user="root")
        try:
            self.db_manager.connect_database()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the database: {e}")

        # YOLO 모델 불러오기
        self.model = load_yolo_model()
        self.bridge = CvBridge()
        
        self.arrive = False
        self.send_state = False
        self.lock = threading.Lock()

        # 초기 이미지 생성 (예: 640x480, 검정색 이미지)
        initial_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.cv_img = initial_image

        # 카메라 토픽 구독자 생성
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            'Admin_Manager/camera/compressed',
            self.camera_callback,
            10
        )

        # 로봇 상태 토픽 구독자 생성 
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            'Admin_Manager/robot_state',
            self.robot_state_callback,
            10
        )

        # 사용자 음성 토픽 구독자 생성 
        self.user_voice_subscription = self.create_subscription(
            Voice,
            'Admin_Manager/recognized_text',
            self.user_voice_callback,
            10
        )

        # 로봇 명령 서비스 클라이언트 생성 
        self.robot_command_client = self.create_client(CommandString, '/robot_command')

        # 순찰 명령 서비스 클라이언트 생성 
        self.patrol_command_client = self.create_client(CommandString, '/patrol_command')

    def detect_people(self, img):
        self.get_logger().info('Detecting people in the image')
        results = self.model(img)
        detected_objects = results.pandas().xyxy[0]
        people_detected = detected_objects[detected_objects['name'] == 'person']
        self.get_logger().info(f'Number of people detected: {len(people_detected)}')
        return len(people_detected) > 0

    def camera_callback(self, msg):
        # 압축된 이미지를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def robot_state_callback(self, msg):
        with self.lock:
            self.robot_state = msg.command  # 메시지 구조에 맞게 수정 필요
            self.get_logger().info(f'Received /robot_state: {self.robot_state}')
            if 'arrive' in self.robot_state:
                self.arrive = True
                self.name = self.robot_state[len("arrive at"):].strip()
                self.check_state()
            elif 'guide' in self.robot_state:
                self.name = self.robot_state[len("guide at"):].strip()
            elif 'not' in self.robot_state:
                self.arrive = True
                self.send_state = True
                self.check_state()
            else:
                self.arrive = False
                self.send_state = False

    def check_state(self):
        if self.arrive:
            if "2" in self.robot_state or "5" in self.robot_state:
                self.send_patrol_command("patrol")
                self.send_state = True
            else:
                if self.detect_people(self.cv_img) and not self.send_state:
                    self.send_robot_command("human_detect")
                    self.send_state = True
                else:
                    self.send_patrol_command("patrol")
                    self.send_state = True

    def user_voice_callback(self, msg):
        self.get_logger().info('Received /user_voice')

        text = msg.command
        keywords = ["강아지", "고양이", "갈색말", "초록양"]

        if "설명" in text:
            self.get_logger().info("Description command detected!")
            script = self.db_manager.find_script_by_name(self.name)
            self.db_manager.insert_event_log(self.name, "description")
            self.send_robot_command("description", script)

        elif "안내" in text:
            self.get_logger().info("Guide command detected!")
            keyword_found = False

            for keyword in keywords:
                if keyword in text:
                    self.get_logger().info(f"Extracted artwork name: {keyword}")
                    self.db_manager.insert_event_log(keyword, "guide")
                    self.send_robot_command("guide", keyword)
                    keyword_found = True
                    break

            if not keyword_found:
                self.send_robot_command("again")

        else:
            self.db_manager.insert_event_log(self.name, "comeback")
            self.send_robot_command("comeback")

    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Robot_command service not available')
            return
        
        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.send_robot_command_callback)

    def send_patrol_command(self, command: str, description: str = ""):
        if not self.patrol_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('patrol_command service not available')
            return
            
        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.patrol_command_client.call_async(request)
        future.add_done_callback(self.send_patrol_command_callback)

    def send_robot_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Robot command executed successfully (Admin to Robot): {response.message}')
            else:
                self.get_logger().error(f'Failed to execute robot command (Admin to Robot): {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_patrol_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Patrol command executed successfully (Admin to Robot): {response.message}')
            else:
                self.get_logger().error(f'Failed to execute patrol command (Admin to Robot): {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rp.init(args=args)
    admin_manager = Admin_Manager()
    rp.spin(admin_manager)
    admin_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
