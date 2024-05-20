import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String
import wave
import numpy as np

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            ByteMultiArray,
            'audio_data',
            self.audio_data_callback,
            10)
        self.info_subscription = self.create_subscription(
            String,
            'audio_info',
            self.audio_info_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.info_subscription  # prevent unused variable warning
        self.audio_data = None
        self.sample_width = None
        self.channels = None
        self.frame_rate = None
        self.received_data = False
        self.received_info = False

    def audio_data_callback(self, msg):
        self.get_logger().info('Audio data received')
        self.audio_data = bytes(msg.data)
        self.received_data = True
        self.check_and_save_audio()

    def audio_info_callback(self, msg):
        self.get_logger().info('Audio info received')
        info = msg.data.split(',')
        self.sample_width = int(info[0])
        self.channels = int(info[1])
        self.frame_rate = int(info[2])
        self.received_info = True
        self.check_and_save_audio()

    def check_and_save_audio(self):
        if self.received_data and self.received_info:
            self.save_audio()
            self.received_data = False
            self.received_info = False

    def save_audio(self):
        self.get_logger().info('Saving audio data...')
        output_path = 'output.wav'
        with wave.open(output_path, 'wb') as output_file:
            output_file.setnchannels(self.channels)
            output_file.setsampwidth(self.sample_width)
            output_file.setframerate(self.frame_rate)
            output_file.writeframes(self.audio_data)
        self.get_logger().info(f'Saved audio to {output_path}')

def main(args=None):
    rclpy.init(args=args)
    audio_subscriber = AudioSubscriber()
    rclpy.spin(audio_subscriber)
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()