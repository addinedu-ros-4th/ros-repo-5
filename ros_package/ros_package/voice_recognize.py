import os
import rclpy
from rclpy.node import Node
from ros_package_msgs.msg import Voice
from ros_package_msgs.srv import CommandString
from ros_package.voice_recorder import VoiceRecorder
from ros_package.sp_recog import audio_to_text
import threading
import time
import sys

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        self.recorder = VoiceRecorder()
        self.publisher_ = self.create_publisher(Voice, '/recognized_text', 10)
        self.running = False
        self.recording_thread = None

        # Queue for storing recognized texts
        self.text_queue = []

        # Stop callback 설정
        self.recorder.set_stop_callback(self.stop_threads)

        # 스레드 초기화
        self.monitor_thread = threading.Thread(target=self.monitor_file_system)
        self.publish_thread = threading.Thread(target=self.publish_recognized_text)

        # 스레드 시작
        self.monitor_thread.start()
        self.publish_thread.start()

    def stop_threads(self):
        """녹음이 중지될 때 스레드를 멈추는 함수"""
        if self.running:
            self.running = False
            if self.recording_thread and self.recording_thread.is_alive():
                recording_thread_copy = self.recording_thread
                self.recording_thread = None
                if threading.current_thread() != recording_thread_copy:
                    recording_thread_copy.join()
                print("Recording thread stopped.")
            self.recorder.stop_recording()

    def record_and_process(self):
        """녹음 및 처리 스레드"""
        self.running = True
        self.recording_thread = threading.Thread(target=self._record_and_process)
        self.recording_thread.start()

    def _record_and_process(self):
        try:
            frames = self.recorder.start_recording(duration=4)
            max_recording_time = 10
            start_time = time.time()

            while rclpy.ok() and self.running:
                if time.time() - start_time > max_recording_time:
                    print("Recording time limit reached! Saving recording...")
                    self.recorder.stop_recording()
                    self.running = False
                    break
                time.sleep(1)

            print("Recording thread stopped.")
            self.recorder.stop_recording()
            self.process_audio_files()

        except OSError as e:
            print(f"Recording failed: {e}")
            self.stop_threads()

    def process_audio_files(self):
        """음성을 처리하고 텍스트를 큐에 추가하는 함수"""
        directory = "/home/jongchanjang/my_mobile/src/ros_package/resource/mic_data"
        if os.listdir(directory):
            latest_file = sorted(os.listdir(directory))[-1]
            latest_file_path = os.path.join(directory, latest_file)
            print("Processing file:", latest_file_path)
            text = audio_to_text(latest_file_path)
            if text:
                print("Recognized text:", text)
                self.text_queue.append(text)
            else:
                print(f"Speech recognition could not understand audio: {latest_file_path}")
            if os.path.exists(latest_file_path):
                os.remove(latest_file_path)

    def monitor_file_system(self):
        """파일 시스템 모니터링 스레드"""
        directory = "/home/jongchanjang/my_mobile/src/ros_package/resource/mic_data"
        print("Monitoring directory:", directory)
        while rclpy.ok():
            if os.listdir(directory):
                latest_file = sorted(os.listdir(directory))[-1]
                latest_file_path = os.path.join(directory, latest_file)
                print("Processing file:", latest_file_path)
                text = audio_to_text(latest_file_path)
                if text:
                    print("Recognized text:", text)
                    self.text_queue.append(text)
                else:
                    text = ""
                    print(f"Speech recognition could not understand audio: {latest_file_path}")
                    self.text_queue.append(text)
                if os.path.exists(latest_file_path):
                    os.remove(latest_file_path)
            time.sleep(1)
        print("Monitoring thread stopped.")

    def publish_recognized_text(self):
        """인식된 텍스트를 발행하는 스레드"""
        while rclpy.ok():
            if self.text_queue:
                text = self.text_queue.pop(0)
                self.send_to_ros2(text)
                self.restart_node()  # 텍스트 발행 후 노드 재시작
            time.sleep(1)
        print("Publishing thread stopped.")

    def restart_node(self):
        """노드를 재시작하는 메서드"""
        print("Restarting node...")
        rclpy.shutdown()
        os.execv(sys.executable, ['python3'] + sys.argv)


    def send_to_ros2(self, text):
        """ROS 2 메시지를 발행하는 함수"""
        msg = Voice()
        msg.command = text
        self.publisher_.publish(msg)

class VoiceNode(Node):
    def __init__(self):
        super().__init__('unique_voice_input_node_name')
        self.voice_recognition_node = VoiceRecognitionNode()
        self.voice_signal_server = self.create_service(CommandString, '/voice_signal', self.voice_signal_callback)

    def voice_signal_callback(self, request, response):
        """Callback function for voice start command"""
        self.get_logger().info('Voice signal received')

        if request.command == "voice_start":
            self.get_logger().info('Processing voice start command')
            if self.voice_recognition_node.running:
                self.voice_recognition_node.stop_threads()
            self.voice_recognition_node.record_and_process()
            response.success = True
            response.message = 'Voice processing started'
        else:
            response.success = False
            response.message = 'Unknown command'

        return response

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