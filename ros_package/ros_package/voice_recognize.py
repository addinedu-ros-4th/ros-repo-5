import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_package_msgs.srv import CommandString
from ros_package.mic_src.voice_recorder import VoiceRecorder
from ros_package.mic_src.audio_amplifier import AudioAmplifier
from ros_package.mic_src.sp_recog import audio_to_text
import signal
import time
import threading

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.recorder = VoiceRecorder()
        signal.signal(signal.SIGINT, self.handle_interrupt)
        self.running = True

        self.voice_signal_server = self.create_service(CommandString, '/voice_signal', self.voice_siganl_callback)

        self.user_voice_publisher_ = self.create_publisher(String, 'user_voice', 10)


    def voice_siganl_callback(self, request, response):
        self.get_logger().info('Voice_siganl Server started')

        if request.command == "voice_start":
            self.get_logger().info('Processing voice_start')
            response.success = True
            response.message = 'voice_start'
            
        elif request.command == "voice_stop":
            self.get_logger().info('Processing voice_stop')
            response.success = True
            response.message = 'voice_stop'

        return response


    def handle_interrupt(self, sig, frame):
        self.recorder.save_recording()
        self.running = False
        exit(0)


    def record_and_process(self):
        while self.running:
            # 음성 녹음 시작
            self.recorder.start_recording()


    def monitor_file_system(self):
        directory = "/home/pink/pinkbot/src/ros_package/resource/mic_data"
        print(directory)
        while self.running:
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


    def send_to_ros2(self, text):
        msg = String()
        msg.data = text
        self.user_voice_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()
    
    # record_and_process 메서드를 실행하는 스레드 시작
    record_thread = threading.Thread(target=voice_node.record_and_process)
    record_thread.start()
    
    # monitor_file_system 메서드를 실행하는 스레드 시작
    monitor_thread = threading.Thread(target=voice_node.monitor_file_system)
    monitor_thread.start()
    
    # 모든 스레드가 종료할 때까지 대기
    record_thread.join()
    monitor_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()