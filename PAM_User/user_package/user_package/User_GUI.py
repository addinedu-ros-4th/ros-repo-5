import sys
import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_package_msgs.srv import CommandString
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QDialog, QLabel
from PyQt6.QtGui import QPixmap, QImage, QTransform
from PyQt6.QtCore import pyqtSignal, QThread
from PyQt6 import QtCore
import os
import cv2
import numpy as np
from gtts import gTTS
import pygame
import threading

class Ros2PyQtApp(QDialog):
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        uic.loadUi("/home/hj/amr_ws/ROS/src/PAM_User/src/user_package/resource/User_GUI.ui", self)
        self.setWindowTitle("패트와 매트")
        
        # Map 오브젝트를 QLabel로 정의
        self.map_label = self.findChild(QLabel, 'Map')
        
        # 신호와 슬롯 연결
        self.update_image_signal.connect(self.update_map_label)

    def update_map_label(self, map_image):
        height, width = map_image.shape
        bytes_per_line = width
        q_image = QImage(map_image.data, width, height, bytes_per_line, QImage.Format.Format_Grayscale8)
        pixmap = QPixmap.fromImage(q_image)

        label_width = self.map_label.width()
        label_height = self.map_label.height()

        scaled_pixmap = pixmap.scaled(label_width, label_height, aspectRatioMode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        self.map_label.setPixmap(scaled_pixmap)


class UserGUI(Node):
    def __init__(self, gui_app):
        super().__init__('User_GUI')
        self.gui_app = gui_app

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/User_GUI/amcl_pose',
            self.pose_callback,
            10
        )

        self.pose_subscription  # prevent unused variable warning

        # Robot_server로 robot_command 서비스를 받아옴
        self.robot_command_server = self.create_service(CommandString, 'User_GUI/robot_command', self.robot_command_callback)

        # pygame 초기화
        pygame.init()

        # 맵 이미지 로드
        self.map_image = cv2.imread('/home/hj/amr_ws/ROS/src/PAM_Admin/map.pgm', cv2.IMREAD_GRAYSCALE)

        # 맵 정보 설정
        self.resolution = 0.05
        self.origin = [-0.327, -1.71, 0]

        self.prev_robot_pose = None
        self.robot_pose_x = int(-self.origin[0] / self.resolution)
        self.robot_pose_y = int(-self.origin[1] / self.resolution)

        self.update_map_image()

        self.timer = self.create_timer(0.1, self.timer_callback)
    

    def play_tts(self, text):
        tts_thread = threading.Thread(target=self._play_tts, args=(text,))
        tts_thread.start()

    def _play_tts(self, text):
        tts = gTTS(text, lang='ko')
        tts.save('temp.mp3')
        pygame.mixer.music.load('temp.mp3')
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        os.remove('temp.mp3')

    def robot_command_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        if request.command == "human_detect":
            self.process_command(response, 'human_detect', '무엇을 도와드릴까요?')

        elif request.command == "description":
            self.process_command(response, 'description', f'작품설명 하겠습니다 {request.description}, 또 다른 도움이 필요하십니까?')

        elif request.command == "guide":
            self.process_command(response, 'guide', f'{request.description}로 길안내 시작하겠습니다')

        elif request.command == "re":
            self.process_command(response, 're', request.description)

        elif request.command == "again":
            self.process_command(response, 'again', '작품이름을 넣어서 다시 한번 말씀해주세요')

        elif request.command == "comeback":
            self.process_command(response, 'comeback', '편안한 관람되십시오')

        else:
            self.get_logger().error('Unknown command received')
            response.success = False
            response.message = 'Unknown'

        return response

    def process_command(self, response, message, tts_text):
        self.get_logger().info(f'Processing {message}')
        response.success = True
        response.message = message
        self.play_tts(tts_text)
    
    def pose_callback(self, msg):
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)

    def update_map_image(self):
        map_height = self.map_image.shape[0]

        if self.prev_robot_pose is not None:
            cv2.circle(self.map_image, self.prev_robot_pose, 3, (255, 255, 255), -1)

        robot_image_y = map_height - self.robot_pose_y
        cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 3, (0, 255, 0), -1)
        self.prev_robot_pose = (self.robot_pose_x, robot_image_y)

        self.gui_app.update_image_signal.emit(self.map_image)

    def timer_callback(self):
        self.update_map_image()


class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rp.spin(self.node)


def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)
    ros2_pyqt_app = Ros2PyQtApp()

    user_gui = UserGUI(ros2_pyqt_app)

    ros2_thread = Ros2Thread(user_gui)
    ros2_thread.start()

    ros2_pyqt_app.show()
    app.exec()

    user_gui.destroy_node()
    rp.shutdown()
    ros2_thread.wait()

if __name__ == '__main__':
    main()

