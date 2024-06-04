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

class VoiceRecognize:
    def __init__(self):
        # 음성 녹음을 위한 VoiceRecorder 인스턴스 생성
        self.recorder = VoiceRecorder()
        # 녹음 및 처리 상태를 나타내는 플래그
        self.running = False
        # 녹음 및 처리를 담당하는 스레드
        self.recording_thread = None
        # 파일 모니터링을 담당하는 스레드
        self.file_monitor_thread = None
        # 인식된 텍스트를 저장하는 큐
        self.text_queue = []
        # 큐에 새로운 텍스트가 추가될 때 알림을 위한 조건 변수
        self.text_queue_condition = threading.Condition()
        # 텍스트를 발행하는 Publisher 생성
        self.publisher_ = None
    def start(self, publisher):
        # 녹음 및 처리를 시작하는 메서드
        if not self.running:
            self.publisher_ = publisher
            self.running = True
            self.recording_thread = threading.Thread(target=self._record_and_process)
            self.file_monitor_thread = threading.Thread(target=self._start_file_monitoring)
            self.recording_thread.start()
            self.file_monitor_thread.start()
    def stop(self):
        # 녹음 및 처리를 중지하는 메서드
        self.running = False
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join()
        if self.file_monitor_thread and self.file_monitor_thread.is_alive():
            self.file_monitor_thread.join()
        self.recorder.stop_recording()
    def _start_file_monitoring(self):
        # 파일 모니터링을 시작하는 메서드
        directory = "/home/hj_rpi/PAM_Robot/src/ros_package/resource/mic_data"
        while self.running:
            if os.listdir(directory):
                self.process_audio_files()
            time.sleep(1)
    def _record_and_process(self):
        try:
            # 음성 녹음 시작
            frames = self.recorder.start_recording(duration=4)
            max_recording_time = 10
            start_time = time.time()
            while rclpy.ok() and self.running:
                if time.time() - start_time > max_recording_time:
                    # 녹음 시간 제한에 도달하면 녹음 중지
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
            self.stop()
    def process_audio_files(self):
        directory = "/home/hj_rpi/PAM_Robot/src/ros_package/resource/mic_data"
        if os.listdir(directory):
            latest_file = sorted(os.listdir(directory))[-1]
            latest_file_path = os.path.join(directory, latest_file)
            print("Processing file:", latest_file_path)
            # 오디오 파일을 텍스트로 변환
            text = audio_to_text(latest_file_path)
            if text is not None:
                print("Recognized text:", text)
                self.text_queue.append(text)
                # 발행
                self.send_to_ros2(text)
            else:
                print(f"Speech recognition could not understand audio: {latest_file_path}")
                self.text_queue.append("")
                # 발행
                self.send_to_ros2("")
            if os.path.exists(latest_file_path):
                os.remove(latest_file_path)
            with self.text_queue_condition:
                # 새로운 텍스트가 큐에 추가되었음을 알림
                self.text_queue_condition.notify_all()
    def send_to_ros2(self, text):
        """ROS 2 메시지를 발행하는 함수"""
        msg = Voice()
        msg.command = text
        self.publisher_.publish(msg)
        
        
class VoiceNode(Node):
    def __init__(self):
        super().__init__('unique_voice_input_node_name')
        # 음성 인식 노드 인스턴스 초기화
        self.voice_recognition_node = None
        # voice_signal 서비스를 생성
        self.voice_signal_server = self.create_service(CommandString, '/voice_signal', self.voice_signal_callback)
        # 결과 텍스트를 발행하는 Publisher 생성
        self.publisher_ = self.create_publisher(Voice, '/recognized_text', 10)
    def voice_signal_callback(self, request, response):
        self.get_logger().info('Voice signal received')
        if request.command == "voice_start":
            # voice_start 요청이 들어오면 음성 인식 노드 시작
            self.get_logger().info('Processing voice start command')
            if self.voice_recognition_node and self.voice_recognition_node.running:
                # 이미 실행 중인 음성 인식 노드가 있으면 중지
                self.voice_recognition_node.stop()
            # 새로운 음성 인식 노드 인스턴스 생성 및 시작
            self.voice_recognition_node = VoiceRecognize()
            self.voice_recognition_node.start(self.publisher_)
            response.success = True
            response.message = 'Voice processing started'
        else:
            response.success = False
            response.message = 'Unknown command'
        return response
def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    # VoiceNode 객체 생성
    voice_node = VoiceNode()
    try:
        # 노드 실행
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    # 노드 종료 시 음성 인식 노드 중지 및 ROS 2 종료
    if voice_node.voice_recognition_node:
        voice_node.voice_recognition_node.stop()
    voice_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()