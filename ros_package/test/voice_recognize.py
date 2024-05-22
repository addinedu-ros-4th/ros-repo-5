import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_package.voice_recorder import VoiceRecorder
from ros_package.audio_amplifier import AudioAmplifier
from ros_package.sp_recog import audio_to_text
from ros_package.thread_manager import RecordingThread, MonitoringThread
import signal
import time
import threading

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.recorder = VoiceRecorder()
        signal.signal(signal.SIGINT, self.handle_interrupt)
        self.publisher_ = self.create_publisher(String, 'recognized_text', 10)
        
        # record_and_process 메서드를 실행하는 스레드 시작
        self.record_thread = threading.Thread(target=self.record_and_process)
        self.record_thread.start()
        
        # monitor_file_system 메서드를 실행하는 스레드 시작
        self.monitor_thread = threading.Thread(target=self.monitor_file_system)
        self.monitor_thread.start()

    def handle_interrupt(self, sig, frame):
        self.recorder.save_recording()
        self.destroy_node()  # 노드 종료
        exit(0)

    def record_and_process(self):
        # 처음 4초 동안 녹음 수행
        self.recorder.start_recording(duration=4)
        # 음성이 인식될 때까지 녹음 및 처리 반복
        while rclpy.ok():
            # 파일이 생성되었는지 확인
            directory = "/home/pink/pinkbot/src/ros_package/ros_package/mic_data"
            current_file_count = len(os.listdir(directory))
            if current_file_count > 0:
                # 파일이 생성되면 녹음 스레드 종료
                break
            # 음성 녹음 시작
            self.recorder.start_recording()
            # 새로운 음성이 생성될 때까지 기다림
            while self.recorder.recording and rclpy.ok():
                time.sleep(1)
            # 새로운 음성이 생성되면 스레드 종료
            self.recorder.stop_recording()

        # 파일이 생성되었을 때 녹음 스레드가 종료되도록 처리
        if current_file_count > 0:
            self.destroy_node()  # 노드 종료
        # 스레드가 종료되었음을 알리기 위해 메시지 출력
        print("Recording thread stopped.")

    def monitor_file_system(self):
        directory = "/home/pink/pinkbot/src/ros_package/ros_package/mic_data"
        print(directory)
        while rclpy.ok():
            print("Inside the while loop")  # 루프가 실행되는지 확인
            current_file_count = len(os.listdir(directory))
            if current_file_count > 0:
                # 파일이 존재할 때만 처리
                latest_file = sorted(os.listdir(directory))[-1]
                latest_file_path = os.path.join(directory, latest_file)
                print("Processing file:", latest_file_path)  # 디버그 문 추가
                text = audio_to_text(latest_file_path)
                if text:
                    print("Recognized text:", text)
                    self.send_to_ros2(text)
                # 파일 삭제
                os.remove(latest_file_path)
            time.sleep(1)  # 1초 대기
        # 스레드가 종료되었음을 알리기 위해 메시지 출력
        print("Monitoring thread stopped.")

    def send_to_ros2(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    
    # 모든 스레드가 종료할 때까지 대기
    main_node.record_thread.join()
    main_node.monitor_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()