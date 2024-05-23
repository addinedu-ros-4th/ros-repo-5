import rclpy
from rclpy.node import Node
from msg_pkg.msg import SignalMsg

class MsgPublisher(Node):
    def __init__(self):
        super().__init__('msg_publisher')
        self.publisher_ = self.create_publisher(SignalMsg, 'signal_topic', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = SignalMsg()
        msg.recv_msg = 'Hello, ROS 2'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.recv_msg}"')

def main(args=None):
    rclpy.init(args=args)
    node = MsgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()