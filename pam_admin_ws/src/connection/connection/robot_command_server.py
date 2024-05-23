import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class RobotCommandServer(Node):
    def __init__(self):
        super().__init__('robot_command_server')
        self.srv = self.create_service(SetBool, 'robot_command', self.command_callback)
        self.get_logger().info('Robot Command Server started')

    def command_callback(self, request, response):
        if request.data:
            self.get_logger().info('Received command to start action')
            # 여기에 실제 로봇 동작 코드를 추가
            response.success = True
            response.message = 'Action started'
        else:
            self.get_logger().info('Received command to stop action')
            # 여기에 실제 로봇 정지 코드를 추가
            response.success = True
            response.message = 'Action stopped'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()