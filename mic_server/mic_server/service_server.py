import rclpy
from rclpy.node import Node
from mic_msgs.srv import SetString

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetString, 'set_string', self.set_string_callback)

    def set_string_callback(self, request, response):
        self.get_logger().info('Incoming request: %s' % request.data)
        response.result = len(request.data)
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()