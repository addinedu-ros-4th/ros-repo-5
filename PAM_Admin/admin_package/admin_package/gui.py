import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from ros_package_msgs.msg import RobotState
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QTextEdit, QPushButton, QComboBox, QDateEdit, QVBoxLayout, QTableWidget, QTableWidgetItem
from PyQt6.QtGui import QPixmap, QImage
from cv_bridge import CvBridge
import cv2
from PyQt6 import uic
from threading import Thread
from PyQt6.QtCore import pyqtSignal, QObject
from ros_package_msgs.srv import CommandString 
from .DB_Manager import DatabaseManager

class Ros2PyQtApp(QMainWindow):
    
    update_state_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        uic.loadUi("./gui.ui", self)
        self.setWindowTitle("ROS2 PyQt6 Image Viewer")
        self.update_state_signal.connect(self.setState)  # 신호와 슬롯 연결

        # self.send_msg_btn.clicked.connect(self.send_message)
        self.node = rclpy.create_node('admin_node')
        # self.publisher = self.node.create_publisher(SignalMsg, 'state', 10)

        self.person_flag = False
        self.detect_array = []

        self.robot_command_client = self.node.create_client(CommandString, '/robot_command')
        #DB연결
        self.db_manager = DatabaseManager(host="localhost", user="root")
        #search 버튼 클릭
        self.search_btn.clicked.connect(self.search_event_logs)

        # QDateEdit 설정
        self.dateStart.setDisplayFormat("yyyy-MM-dd")
        self.dateEnd.setDisplayFormat("yyyy-MM-dd")
        
    def setState(self):
        if self.person_flag == True:
            self.state_textEdit.setText("Person Detected!")
            return
        else :
            self.state_textEdit.setText("Normal")
            return
        
    # def send_message(self):
    #     req = SignalMsg()
    #     req.recv_msg = "Send_msg"
    #     self.publisher.publish(req)

        
        
    def display_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.camera_label.setPixmap(QPixmap.fromImage(q_image))
        
    def display_map(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.map_label.setPixmap(QPixmap.fromImage(q_image))
        
    def update_text_edit(self, text):
        if text == "Person detected!":
            self.person_flag = True
            self.detect_array.append("person")
            print(self.detect_array)

            if len(self.detect_array) > 10:
                self.log_textEdit.append("Person Detected!")  # scale 값을 log_textEdit에 추가로 기록
                self.detect_array.clear()  # detect_array 초기화
                self.send_robot_command("human_detect")
            self.update_state_signal.emit()  # 신호 발생
        else :
            self.person_flag = False
            self.detect_array.clear()  # detect_array 초기화            
            self.update_state_signal.emit()  # 신호 발생

    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Robot_command service not available')
            return
        
        request = CommandString.Request()
        request.command = command
        request.description = description
        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.send_robot_command_callback)

    
    def send_robot_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info('Robot command executed successfully (Admin to Robot): %s' % response.message)
            else:
                self.node.get_logger().error('Failed to execute robot command (Admin to Robot): %s' % response.message)
        except Exception as e:
            self.node.get_logger().error('Service call failed: %s' % str(e))

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
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.ui_app.display_image(cv_image)
        
class MapSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('map_subscriber')
        self.ui_app = ui_app
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            'amcl_map',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.ui_app.display_map(cv_image)

class TextSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('text_subscriber')
        self.ui_app = ui_app
        
        self.subscription = self.create_subscription(
            RobotState,
            'signal_topic',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        self.ui_app.update_text_edit(msg.recv_msg)
        
class StateSubscriber(Node):
    def __init__(self, ui_app):
        super().__init__('state_subscriber')
        self.ui_app = ui_app
        
        self.subscription = self.create_subscription(
            RobotState,
            'Admin_Manager/robot_state',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        self.ui_app.update_text_edit(msg.command)

        
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ros2_pyqt_app = Ros2PyQtApp()
    ros2_pyqt_app.show()

    executor = MultiThreadedExecutor()
    
    image_subscriber = ImageSubscriber(ros2_pyqt_app)
    map_subscriber = MapSubscriber(ros2_pyqt_app)
    text_subscriber = TextSubscriber(ros2_pyqt_app)
    state_subscriber = StateSubscriber(ros2_pyqt_app)
    
    executor.add_node(image_subscriber)
    executor.add_node(map_subscriber)
    executor.add_node(text_subscriber)
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
