import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_package_msgs.srv import CommandString
from ros_package.mic_src.voice_recorder import VoiceRecorder
from ros_package.mic_src.sp_recog import audio_to_text
import threading
import time

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.recorder = VoiceRecorder()
        self.publisher_ = self.create_publisher(String, '/recognized_text', 10)
        self.running = False
        self.recording_thread = None  # 녹음 스레드 참조를 저장하는 변수

        # Service to receive voice start command
        self.voice_signal_server = self.create_service(CommandString, '/voice_signal', self.voice_signal_callback)

        # Stop callback 설정
        self.recorder.set_stop_callback(self.stop_threads)

        # 스레드 초기화
        self.monitor_thread = threading.Thread(target=self.monitor_file_system)

        # 스레드 시작
        self.monitor_thread.start()

    def voice_signal_callback(self, request, response):
        """Callback function for voice start command"""
        self.get_logger().info('Voice signal received')

        if request.command == "voice_start":
            self.get_logger().info('Processing voice start command')
            # Stop the current recording thread if it's running
            if self.running:
                self.running = False
                self.recording_thread.join()  # Wait for the thread to finish
            # Start a new recording thread
            self.recording_thread = threading.Thread(target=self.record_and_process)
            self.recording_thread.start()
            self.running = True
            response.success = True
            response.message = 'Recording started'
        else:
            response.success = False
            response.message = 'Unknown command'

        return response

    def stop_threads(self):
        """녹음이 중지될 때 스레드를 멈추는 함수"""
        self.running = False

    def record_and_process(self):
        """녹음 및 처리 스레드"""
        self.recorder.start_recording(duration=4)
        max_recording_time = 10
        start_time = time.time()

        while rclpy.ok() and self.running:
            if time.time() - start_time > max_recording_time:
                print("Recording time limit reached! Saving recording...")
                self.recorder.stop_recording()
                self.running = False  # 녹음이 완료되면 녹음 상태를 재설정
                break

            time.sleep(1)

        print("Recording thread stopped.")


    def monitor_file_system(self):
        """파일 시스템 모니터링 스레드"""
        directory = "/home/pink/pinkbot/src/ros_package/ros_package/mic_data"
        print("Monitoring directory:", directory)
        while rclpy.ok():
            if os.listdir(directory):
                latest_file = sorted(os.listdir(directory))[-1]
                latest_file_path = os.path.join(directory, latest_file)
                print("Processing file:", latest_file_path)
                text = audio_to_text(latest_file_path)
                if text:
                    print("Recognized text:", text)
                    self.send_to_ros2(text)
                else:
                    print(f"Speech recognition could not understand audio: {latest_file_path}")
                os.remove(latest_file_path)

            time.sleep(1)

        print("Monitoring thread stopped.")

    def send_to_ros2(self, text):
        """ROS 2 메시지를 발행하는 함수"""
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    voice_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
