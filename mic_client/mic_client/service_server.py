import rclpy
from rclpy.node import Node
import wave
from std_msgs.msg import ByteMultiArray, String

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(ByteMultiArray, 'audio_data', 10)
        self.info_publisher = self.create_publisher(String, 'audio_info', 10)
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_audio)
        self.file_path = '/home/ito/ros_final/src/mic_client/mic_client/hello.wav'
        self.get_logger().info('Audio Publisher Initialized')

    def publish_audio(self):
        try:
            with wave.open(self.file_path, 'rb') as wav_file:
                frames = wav_file.getnframes()
                audio_data = wav_file.readframes(frames)

                # Publish audio data
                msg = ByteMultiArray()
                msg.data = bytearray(audio_data)
                self.publisher_.publish(msg)

                # Publish audio info
                info_msg = String()
                info_msg.data = f'{wav_file.getsampwidth()},{wav_file.getnchannels()},{wav_file.getframerate()}'
                self.info_publisher.publish(info_msg)

                self.get_logger().info('Audio data published')

        except wave.Error as e:
            self.get_logger().error(f'Wave error: {e}')
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {self.file_path}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()
    rclpy.spin(audio_publisher)
    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
