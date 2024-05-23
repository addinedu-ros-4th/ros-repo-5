import sys
import rclpy
from rclpy.node import Node
from msg_pkg.msg import SignalMsg
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QMainWindow
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6 import uic
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from_class = uic.loadUiType("/home/hj/amr_ws/ROS/src/Ros_final/src/ui_pkg/ui_pkg/msg.ui")[0]

class SignalEmitter(QObject):
    # PyQt6에서 사용할 신호 정의
    text_signal = pyqtSignal(str)
    
class MsgSubscriber(Node):
    def __init__(self, signal_emitter):
        super().__init__('msg_subscriber')
        self.signal_emitter = signal_emitter
        self.subscription = self.create_subscription(
            SignalMsg,
            'signal_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.signal_emitter.text_signal(msg.recv_msg)
        
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Image Viewer")
        
    #     self.signal_emitter = SignalEmitter()
        
    #     # 신호와 슬롯 연결
    #     self.signal_emitter.text_signal.connect(self.update_text)

    # def update_text(self, text):
    #     # 안전하게 UI 업데이트
    #     self.textEdit.setText(text)
        
def main(args=None):
    # rclpy.init(args=args)
    # executor = rclpy.executors.MultiThreadedExecutor()

    app = QApplication(sys.argv)
    main_window = WindowClass()
    main_window.show()

    # ROS 2 스레드 시작
    # msg_subscriber = MsgSubscriber(main_window.signal_emitter)
    # executor.add_node(msg_subscriber)

    # thread = Thread(target=executor.spin)
    # thread.start()
    
    # try:
    #     sys.exit(app.exec())
    # finally:
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()