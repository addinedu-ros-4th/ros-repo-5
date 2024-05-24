import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/camera/compressed', 10)
        time_period = 0.1  # 0.1초마다 이미지를 전송 (10 FPS)
        self.timer = self.create_timer(time_period, self.time_callback)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device")
            rclpy.shutdown()
            return
        self.cv_bridge = CvBridge()
        
        self.get_logger().info("Camera Start")

    def time_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()

        self.publisher.publish(compressed_img_msg)
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = ImgPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()