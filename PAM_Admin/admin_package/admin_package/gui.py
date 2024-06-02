import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 
from ros_package_msgs.msg import RobotState
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QTableWidgetItem
from PyQt6.QtGui import QPixmap, QImage, QTransform
from PyQt6.QtCore import pyqtSignal, QDate
from PyQt6 import uic
from threading import Thread
from .DB_Manager import DatabaseManager
import cv2
import numpy as np

class Ros2PyQtApp(QMainWindow):
    update_image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        uic.loadUi("/home/hj/amr_ws/ROS/src/PAM_Admin/src/admin_package/admin_package/gui.ui", self)
        self.setWindowTitle("관리자 전용")

        self.update_image_signal.connect(self.update_map_label)  # 신호와 슬롯 연결

        self.node = rclpy.create_node('admin_node')

        self.person_flag = False
        self.detect_array = []

        # DB 연결
        self.db_manager = DatabaseManager(host="localhost", user="root")
        self.db_manager.connect_database()

        # search 버튼 클릭
        self.search_btn.clicked.connect(self.search_event_logs)

        # 현재 날짜를 가져와서 dateEnd 위젯에 설정
        self.dateEnd.setDate(QDate.currentDate())
        self.dateEnd.setDisplayFormat("yyyy-MM-dd")

    def display_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.camera_label.setPixmap(QPixmap.fromImage(q_image))

    def update_map_label(self, map_image):
        height, width = map_image.shape
        bytes_per_line = width
        q_image = QImage(map_image.data, width, height, bytes_per_line, QImage.Format.Format_Grayscale8)
        pixmap = QPixmap.fromImage(q_image)

        label_width = self.map_label.width()
        label_height = self.map_label.height()

        scaled_pixmap = pixmap.transformed(QTransform().rotate(90)).scaled(label_width, label_height, aspectRatioMode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)
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

        # 맵 이미지 로드
        self.map_image = cv2.imread('/home/hj/amr_ws/ROS/src/PAM_Admin/map.pgm', cv2.IMREAD_GRAYSCALE)

        # 맵 정보 설정
        self.resolution = 0.05  # 맵의 해상도
        self.origin = [-0.327, -1.71, 0]  # 맵의 원점

        self.prev_robot_pose = None
        self.robot_pose_x = int(-self.origin[0] / self.resolution)
        self.robot_pose_y = int(-self.origin[1] / self.resolution)

        self.update_map_image()

    def pose_callback(self, msg):
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)
        self.update_map_image()

    def update_map_image(self):
        map_height = self.map_image.shape[0]

        if self.prev_robot_pose is not None:
            cv2.circle(self.map_image, self.prev_robot_pose, 3, (255, 255, 255), -1)

        robot_image_y = map_height - self.robot_pose_y
        cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 3, (0, 255, 0), -1)
        self.prev_robot_pose = (self.robot_pose_x, robot_image_y)

        self.ui_app.update_image_signal.emit(self.map_image)


class StateSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('state_subscriber')
        self.ui_app = ui_app
        self.previous_state = ""

        self.state_label = self.ui_app.state_label
        
        self.subscription = self.create_subscription(
            RobotState,
            'Admin_Manager/robot_state',
            self.state_callback,
            10)

    def state_callback(self, msg):
        current_state = msg.command
        
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
