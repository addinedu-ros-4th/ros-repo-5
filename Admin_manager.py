import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from pam_interfaces.srv import CommandString 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from cv_bridge import CvBridge
from pam_interfaces.srv import RobotCommand
from std_msgs.msg import String
from std_srvs.srv import SetBool
import torch
import cv2
import numpy as np
from pam_interfaces.srv import CommandString 
from pam_interfaces.msg import RobotState

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

class Admin_Manager(Node):
    def __init__(self):
        super().__init__('Admin_Manager')
        

        # YOLO 모델 불러오기
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        self.arrive = False
        self.send_state = False   
        # 초기 이미지 생성 (예: 640x480, 검정색 이미지)
        initial_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.cv_img = initial_image     
    #   이미지를 YOLO 모델로 처리 및 사람 검출
    

        # camera 토픽 구독자 생성
        self.camera_subscription = self.create_subscription(
            Image,
            '/Admin_Manager/camera',
            self.camera_callback,
            10
        )

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/Robot_Topic/Admin_Manager/amcl_pose',
            self.pose_callback,
            10
        )

        # robot_state 토픽 구독자 생성 
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            '/Admin_Manager/robot_state',
            self.robot_state_callback,
            10
        )

        # user_voice 토픽 구독자 생성 
        self.user_voice_subscription = self.create_subscription(
            RobotState,
            '/Robot_Topic/Admin_Manager/user_voice',
            self.user_voice_callback,
            10
        )

        #  /robot_command 서비스 보내기 
        self.robot_command_client = self.create_client(CommandString, '/A/robot_command')
        self.state_check_timer = self.create_timer(1.0, self.check_state)
        
        self.camera_subscription
        self.pose_subscription
        self.robot_state_subscription
        self.user_voice_subscription


    # def detect_people(self, model, img):
    #         self.get_logger().info('DETECTING-DETECTING-DETECTING')

    #         results = model(img)
    #         detected_objects = results.pandas().xyxy[0]
    #         people_detected = detected_objects[detected_objects['name'] == 'person']
    #         print(len(people_detected))

    #         return True if len(people_detected) > 0 else False
        
    def camera_callback(self, msg):
        self.get_logger().info('Received /camera')
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

   


    def pose_callback(self, msg):
        self.get_logger().info('Received /amcl_pose')

        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)


    def robot_state_callback(self, msg):
        self.get_logger().info('Received /robot_state')
        robot_state = msg.command  # 메시지 구조에 맞게 수정 필요
        self.get_logger().info(f'{robot_state}')

        if robot_state == 'arrive':
            self.arrive = True
        else:
            self.arrive = False
            self.send_state = False
        self.get_logger().info(f'Arrive: {self.arrive}')



    def user_voice_callback(self, msg):
        # self.get_logger().info('Received /user_voice')
        user_voice = msg
        

    def check_state(self):
        self.get_logger().info("Checking state")

        if self.arrive:
            self.get_logger().info("arrive is true")

            if detect_people(self.model, self.cv_img) and self.send_state == False:
                self.get_logger().info("Requrest to robot : HD")

                self.send_robot_command("human_detect", "human_detected")
                self.send_state = True
                

            
    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Robot_command service not available')
            

        request = CommandString.Request()
        if command == "human_detect":
            
            request.command = command
            request.description = "human_detected"
        elif command == "voice":
            request.command = command
            request.description = description
        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.send_robot_command_callback)

    
    def send_robot_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot command executed successfully (Admin to Robot): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute robot command (Admin to Robot): %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


def main(args=None):
    rp.init(args=args)

    admin_manager = Admin_Manager()


    rp.spin(admin_manager)

    admin_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()