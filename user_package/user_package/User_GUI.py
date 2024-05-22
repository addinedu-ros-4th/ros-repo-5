import sys
import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QDialog, QLabel
from PyQt5.QtGui import QPixmap, QImage , QTransform
from PyQt5.QtCore import pyqtSignal, Qt, QThread
import cv2
import numpy as np

class Ros2PyQtApp(QDialog):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        uic.loadUi("/home/jongchanjang/my_mobile/src/user_package/resource/User_GUI.ui", self)
        self.setWindowTitle("패트와 매트")
        
        # Map 오브젝트를 QLabel로 정의
        self.map_label = self.findChild(QLabel, 'Map')
        
        # 신호와 슬롯 연결
        self.update_image_signal.connect(self.update_map_label)

    def update_map_label(self, map_image):
        # numpy 배열을 QImage로 변환
        height, width = map_image.shape
        bytes_per_line = width
        q_image = QImage(map_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
        
        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(q_image)

        # 이미지를 왼쪽으로 90도 회전
        transformed_pixmap = pixmap.transformed(QTransform().rotate(-90))

        # QLabel에 이미지 설정
        self.map_label.setPixmap(transformed_pixmap)


class UserGUI(Node):
    def __init__(self, gui_app):
        super().__init__('User_GUI')
        self.gui_app = gui_app

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'User_GUI/amcl_pose',
            self.pose_callback,
            10
        )

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
        self.timer = self.create_timer(1.0, self.timer_callback)


    def pose_callback(self, msg):
        self.get_logger().info('Received /amcl_pose')

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
        self.gui_app.update_image_signal.emit(self.map_image)

    def timer_callback(self):
        # 위치를 주기적으로 업데이트
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
    app.exec_()

    user_gui.destroy_node()
    rp.shutdown()
    ros2_thread.wait()

if __name__ == '__main__':
    main()
