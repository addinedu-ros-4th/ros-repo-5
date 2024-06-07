import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 
from ros_package_msgs.msg import RobotState
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QTextEdit, QPushButton, QComboBox, QDateEdit, QVBoxLayout, QTableWidget, QTableWidgetItem
from PyQt6.QtGui import QPixmap, QImage, QTransform
from cv_bridge import CvBridge
import cv2
from PyQt6 import uic
from threading import Thread
from PyQt6.QtCore import pyqtSignal, QObject , QDate
from PyQt6 import QtCore
# from ros_package_msgs.srv import CommandString 
from .DB_Manager import DatabaseManager
from datetime import datetime
import os
import numpy as np
from gtts import gTTS
import pygame

class Ros2PyQtApp(QMainWindow):
    
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        uic.loadUi("/home/jongchanjang/pinkbot/src/PAM_Admin/admin_package/resource/gui.ui", self)
        self.setWindowTitle("관리자 전용")

        self.update_image_signal.connect(self.update_map_label)  # 신호와 슬롯 연결

        self.node = rclpy.create_node('admin_node')

        self.person_flag = False
        self.detect_array = []

        # pygame 초기화
        pygame.init()

        #DB연결
        self.db_manager = DatabaseManager(host="localhost", user="root")
        #search 버튼 클릭
        self.search_btn.clicked.connect(self.search_event_logs)

        # 현재 날짜를 가져와서 dateEnd 위젯에 설정
        self.dateEnd.setDate(QDate.currentDate())
        self.dateEnd.setDisplayFormat("yyyy-MM-dd")

        # Map 오브젝트를 QLabel로 정의
        self.map_label = self.findChild(QLabel, 'map_label')

        # camera 오브젝트를 Qlabel로 정의
        self.camera_label = self.findChild(QLabel, 'camera_label')


    def play_tts(self, text):
        # TTS 생성
        tts = gTTS(text, lang='ko')

        # 임시 파일로 저장
        tts.save('temp.mp3')

        # 재생
        pygame.mixer.music.load('temp.mp3')
        pygame.mixer.music.play()

        # 재생이 완료될 때까지 대기
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        # 임시 파일 삭제
        os.remove('temp.mp3')
        

    def display_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()

        # QLabel의 크기 가져오기
        label_width = self.camera_label.width()
        label_height = self.camera_label.height()

        # QPixmap을 QLabel 크기에 맞게 조정
        scaled_pixmap = q_image.scaled(label_width, label_height, aspectRatioMode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        self.camera_label.setPixmap(QPixmap.fromImage(scaled_pixmap))


    def update_map_label(self, map_image):
        # numpy 배열을 QImage로 변환
        height, width, _ = map_image.shape  # 여기서는 채널 수를 무시하면 됩니다.
        bytes_per_line = 3 * width
        q_image = QImage(map_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
    
        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(q_image)


        # 이미지를 왼쪽으로 0도 회전
        transformed_pixmap = pixmap.transformed(QTransform().rotate(0))

        # QLabel의 크기 가져오기
        label_width = self.map_label.width()
        label_height = self.map_label.height()

        # QPixmap을 QLabel 크기에 맞게 조정
        scaled_pixmap = transformed_pixmap.scaled(label_width, label_height, aspectRatioMode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        # QLabel에 이미지 설정
        self.map_label.setPixmap(scaled_pixmap)


    def search_event_logs(self):
        service_name = self.Service.currentText()
        object_name = self.Object.currentText()
        start_date = self.dateStart.date().toString("yyyy-MM-dd")
        end_date = self.dateEnd.date().toString("yyyy-MM-dd")

        logs = self.db_manager.search_event_logs(service_name, object_name, start_date, end_date)
        self.dbTable.setRowCount(0)

        for row_number, row_data in enumerate(logs):
            self.dbTable.insertRow(row_number)
            for column_number, data in enumerate(row_data):
                self.dbTable.setItem(row_number, column_number, QTableWidgetItem(str(data)))
            
            
class ImageSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('image_subscriber')
        self.ui_app = ui_app
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            CompressedImage,
            'Admin_Manager/camera/compressed',
            self.image_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.ui_app.display_image(cv_image)
        

class MapSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('map_subscriber')
        self.ui_app = ui_app
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'Admin_Manager/amcl_pose',
            self.pose_callback,
            10)

        self.subscription  # prevent unused variable warning

        # 맵 이미지 로드
        self.original_map_image = cv2.imread('/home/jongchanjang/pinkbot/src/maps/Admin_map.png', cv2.IMREAD_COLOR)

        # 맵 정보 설정
        self.resolution = 0.0064  # 맵의 해상도
        self.origin = [-0.65, -1, 0]  # 맵의 원점

        # 이전 로봇 위치
        self.prev_robot_pose = None

        # 초기 로봇 위치 설정
        self.robot_pose_x = int((-self.origin[0] / self.resolution))
        self.robot_pose_y = int((-self.origin[1] / self.resolution))

        self.update_map_image()

        # 주기적으로 위치를 업데이트하는 타이머 설정
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        # self.get_logger().info('Received /amcl_pose')

        print(msg.pose.pose.position)

        # 로봇의 위치 받아오기
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  # x 위치
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)  # y 위치

    def update_map_image(self):
        # 원본 맵 이미지를 복사하여 현재 맵 이미지로 설정
        self.map_image = self.original_map_image.copy()

        # BGR에서 RGB로 색상 공간 변환
        self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2RGB)

        # 맵 이미지의 높이
        map_height = self.map_image.shape[0]

        # 로봇 위치를 이미지 좌표로 변환 (상하 반전)
        robot_image_y = map_height - self.robot_pose_y

        # 맵 이미지에 새로운 로봇 위치 표시
        cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 10, (0, 255, 0), -1)

        # 이전 로봇 위치 업데이트
        self.prev_robot_pose = (self.robot_pose_x, robot_image_y)

        # 신호를 통해 GUI 업데이트 요청
        self.ui_app.update_image_signal.emit(self.map_image)

    def timer_callback(self):
        # 위치를 주기적으로 업데이트
        self.update_map_image()



class StateSubscriber(Node):

    def __init__(self, ui_app):
        super().__init__('state_subscriber')
        self.ui_app = ui_app
        self.previous_state = ""  # 이전 상태 저장 변수

        self.state_label = self.ui_app.findChild(QLabel, 'state_label')  # QLabel 찾기
        
        self.subscription = self.create_subscription(
            RobotState,
            'Admin_Manager/robot_state',
            self.state_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def state_callback(self, msg):
        current_state = msg.command
        
        # 현재 상태와 이전 상태를 비교하여 변경된 경우에만 텍스트를 업데이트
        if current_state != self.previous_state:
            self.ui_app.state_label.setText(current_state)
            self.previous_state = current_state



class TheftDetector(Node):
    def __init__(self, ui_app):
        super().__init__('theft_detector')
        self.ui_app = ui_app

        self.subscription = self.create_subscription(
            RobotState,
            '/art_theft',
            self.theft_callback,
            10)

        self.end_theft_publisher = self.create_publisher(
            RobotState,
            '/end_theft',
            10)

        self.subscription  # prevent unused variable warning

        # END 버튼을 누르면 도난상황 종료 알림
        self.end_simul = self.ui_app.findChild(QPushButton, 'theft') 
        self.end_simul.clicked.connect(self.end_theft) 

    def theft_callback(self, msg):

        art_name = msg.command

        # current_state 업데이트: 도난 상황
        self.ui_app.state_label.setText('{} 작품 도난 상황 발생'.format(art_name))

        # TTS 실행: 도난 상황 발생
        self.ui_app.play_tts('{} 작품 도난 상황이 발생하였습니다.'.format(art_name))

    def end_theft(self):

        # current_state 업데이트: 도난 상황 종료
        self.ui_app.state_label.setText('작품 도난 상황 종료')

        # TTS 실행: 도난 상황 종료
        self.ui_app.play_tts('작품 도난 상황이 종료되었습니다.')

        # 도난 상황 종료를 알리는 메시지 발행
        msg = RobotState()
        msg.command = '도난 상황 종료'
        self.end_theft_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ros2_pyqt_app = Ros2PyQtApp()
    ros2_pyqt_app.show()

    executor = MultiThreadedExecutor()
    
    image_subscriber = ImageSubscriber(ros2_pyqt_app)
    map_subscriber = MapSubscriber(ros2_pyqt_app)
    state_subscriber = StateSubscriber(ros2_pyqt_app)
    theft_detector = TheftDetector(ros2_pyqt_app)
    
    executor.add_node(image_subscriber)
    executor.add_node(map_subscriber)
    executor.add_node(state_subscriber)
    executor.add_node(theft_detector)

    def run_ros_spin():
        executor.spin()

    thread = Thread(target=run_ros_spin)
    thread.start()
    
    exit_code = app.exec()
    rclpy.shutdown()
    thread.join()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()