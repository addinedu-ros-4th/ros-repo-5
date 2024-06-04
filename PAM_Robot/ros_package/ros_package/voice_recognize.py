import os
import threading
import time
import pyaudio
import speech_recognition as sr
from pydub import AudioSegment
import rclpy
from rclpy.node import Node
from ros_package_msgs.msg import Voice
from ros_package_msgs.srv import CommandString

class AudioAmplifier:
    def __init__(self, gain_dB):
        self.gain_dB = gain_dB

    def amplify_audio(self, audio_data):
        audio = AudioSegment(
            data=audio_data,
            sample_width=2,
            frame_rate=16000,
            channels=1
        )
        amplified_audio = audio + self.gain_dB
        return amplified_audio.raw_data

class VoiceRecorder:
    def __init__(self):
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024
        self.is_recording = False
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.frames = []
        self.amplifier = AudioAmplifier(gain_dB=10)

    def start_recording(self, duration=7):
        """시작 녹음 함수"""
        self.frames = []
        self.stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS,
                                      rate=self.RATE, input=True,
                                      frames_per_buffer=self.CHUNK)
        print("Listening...")
        self.is_recording = True

        # Record for `duration` seconds
        start_time = time.time()
        while self.is_recording:
            if time.time() - start_time >= duration:
                print("Recording time limit reached! Processing recording...")
                break
            data = self.stream.read(self.CHUNK)
            self.frames.append(data)

        # Recording stopped
        print("Recording stopped.")
        self.stop_recording()
        return self.process_recording()

    def stop_recording(self):
        """녹음을 중지하고 스트림을 종료"""
        self.is_recording = False
        self.stream.stop_stream()
        self.stream.close()

    def process_recording(self):
        """녹음된 데이터를 처리"""
        raw_audio = b''.join(self.frames)
        amplified_audio = self.amplifier.amplify_audio(raw_audio)
        return amplified_audio

class VoiceRecognize:
    def __init__(self):
        self.recorder = VoiceRecorder()
        self.running = False
        self.recording_thread = None
        self.text_queue = []
        self.text_queue_condition = threading.Condition()
        self.publisher_ = None

    def start(self, publisher):
        if not self.running:
            self.publisher_ = publisher
            self.running = True
            self.recording_thread = threading.Thread(target=self._record_and_process)
            self.recording_thread.start()

    def stop(self):
        self.running = False
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join()
        self.recorder.stop_recording()

    def _record_and_process(self):
        try:
            amplified_audio = self.recorder.start_recording(duration=7)
            # 음성 인식을 비동기 처리
            threading.Thread(target=self._process_and_recognize, args=(amplified_audio,)).start()

        except OSError as e:
            print(f"Recording failed: {e}")
            self.stop()

    def _process_and_recognize(self, audio_data):
        text = self.audio_to_text(audio_data)
        if text:
            self.text_queue.append(text)
            self.send_to_ros2(text)
        else:
            self.text_queue.append("")
            self.send_to_ros2("")

        with self.text_queue_condition:
            self.text_queue_condition.notify_all()
        self.stop()

    def audio_to_text(self, audio_data):
        recognizer = sr.Recognizer()
        audio_segment = sr.AudioData(audio_data, self.recorder.RATE, self.recorder.audio.get_sample_size(self.recorder.FORMAT))
        try:
            text = recognizer.recognize_google(audio_segment, language='ko-KR')
            print("Speech recognition : {}".format(text))
            return text
        except sr.UnknownValueError:
            print("Speech recognition could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service: {e}")
            return None

    def send_to_ros2(self, text):
        msg = Voice()
        msg.command = text
        self.publisher_.publish(msg)

class VoiceNode(Node):
    def __init__(self):
        super().__init__('unique_voice_input_node_name')
        self.voice_recognition_node = None
        self.voice_signal_server = self.create_service(CommandString, '/voice_signal', self.voice_signal_callback)
        self.publisher_ = self.create_publisher(Voice, '/recognized_text', 10)

    def voice_signal_callback(self, request, response):
        self.get_logger().info('Voice signal received')

        if request.command == "voice_start":
            self.get_logger().info('Processing voice start command')
            if self.voice_recognition_node and self.voice_recognition_node.running:
                self.voice_recognition_node.stop()
            self.voice_recognition_node = VoiceRecognize()
            self.voice_recognition_node.start(self.publisher_)
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

    if voice_node.voice_recognition_node:
        voice_node.voice_recognition_node.stop()
    voice_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
