import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
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
import numpy as np

class Ros2PyQtApp(QMainWindow):
    
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        uic.loadUi("/home/jongchanjang/my_mobile/src/admin_package/resource/gui.ui", self)
        self.setWindowTitle("관리자 전용")

        self.update_image_signal.connect(self.update_map_label)  # 신호와 슬롯 연결

        self.node = rclpy.create_node('admin_node')

        self.person_flag = False
        self.detect_array = []

        #DB연결
        self.db_manager = DatabaseManager(host="localhost", user="root")
        #search 버튼 클릭
        self.search_btn.clicked.connect(self.search_event_logs)

        # 현재 날짜를 가져와서 dateEnd 위젯에 설정
        self.dateEnd.setDate(QDate.currentDate())
        self.dateEnd.setDisplayFormat("yyyy-MM-dd")

        # Map 오브젝트를 QLabel로 정의
        self.map_label = self.findChild(QLabel, 'map_label')
        

    def display_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.camera_label.setPixmap(QPixmap.fromImage(q_image))


    def update_map_label(self, map_image):
        # numpy 배열을 QImage로 변환
        height, width = map_image.shape
        bytes_per_line = width
        q_image = QImage(map_image.data, width, height, bytes_per_line, QImage.Format.Format_Grayscale8)
        
        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(q_image)

        # QLabel의 크기 가져오기
        label_width = self.map_label.width()
        label_height = self.map_label.height()

        # QPixmap을 QLabel 크기에 맞게 조정
        scaled_pixmap = pixmap.scaled(label_width, label_height, aspectRatioMode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

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
            Image,
            'Admin_Manager/camera',
            self.image_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
        self.map_image = cv2.imread('/home/jongchanjang/my_mobile/MUSEUM.pgm', cv2.IMREAD_GRAYSCALE)

        # 맵 정보 설정
        self.resolution = 0.05  # 맵의 해상도
        self.origin = [-1.12, -2.57, 0]  # 맵의 원점

        # 이전 로봇 위치
        self.prev_robot_pose = None

        # 초기 로봇 위치 설정
        self.robot_pose_x = int(-self.origin[0] / self.resolution)
        self.robot_pose_y = int(-self.origin[1] / self.resolution)

        self.update_map_image()

        # 주기적으로 위치를 업데이트하는 타이머 설정
        self.timer = self.create_timer(0.1, self.timer_callback)


    def pose_callback(self, msg):
        # self.get_logger().info('Received /amcl_pose')

        # 로봇의 위치 받아오기
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  # x 위치
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)  # y 위치


    def update_map_image(self):
        # 맵 이미지의 높이
        map_height = self.map_image.shape[0]

        # 이전 로봇 위치 지우기
        if self.prev_robot_pose is not None:
            cv2.circle(self.map_image, self.prev_robot_pose, 3, (255, 255, 255), -1)

        # 로봇 위치를 이미지 좌표로 변환 (상하 반전)
        robot_image_y = map_height - self.robot_pose_y

        # 맵 이미지에 새로운 로봇 위치 표시
        cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 3, (0, 255, 0), -1)

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

        
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ros2_pyqt_app = Ros2PyQtApp()
    ros2_pyqt_app.show()

    executor = MultiThreadedExecutor()
    
    image_subscriber = ImageSubscriber(ros2_pyqt_app)
    map_subscriber = MapSubscriber(ros2_pyqt_app)
    state_subscriber = StateSubscriber(ros2_pyqt_app)
    
    executor.add_node(image_subscriber)
    executor.add_node(map_subscriber)
    executor.add_node(state_subscriber)

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
