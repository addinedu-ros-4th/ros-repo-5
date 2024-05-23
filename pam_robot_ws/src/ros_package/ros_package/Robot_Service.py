import rclpy as rp
from rclpy.node import Node
from msg_pkg.srv import CommandString

class RobotService(Node):
    def __init__(self):
        super().__init__('Robot_Service')

        # Admin_Manager로 robot_command 서비스를 받아옴
        self.robot_command_server = self.create_service(CommandString, '/robot_command', self.robot_command_callback)
        
        # User_GUI로 robot_command 서비스를 보냄
        self.robot_command_client = self.create_client(CommandString, 'User_GUI/robot_command')

        # Robot_Driving로 robot_command 서비스를 보냄
        self.robot_command_client_2 = self.create_client(CommandString, 'Robot_Driving/robot_command')

        # User_GUI로 user_command 서비스를 받아옴
        self.user_command_server = self.create_service(CommandString, '/user_command', self.user_commnad_callback)

        # Admin_Manager로 user_command 서비스를 보냄
        self.user_command_client = self.create_client(CommandString, 'Admin_Manager/user_command')


    def robot_command_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        # 명령에 따라 응답 처리
        if request.command == "human_detect":
            self.get_logger().info('Processing human_detect')
            response.success = True
            response.message = 'human_detect'
        elif request.command == "description":
            self.get_logger().info('Processing description')
            response.success = True
            response.message = 'description'
        elif request.command == "guide":
            self.get_logger().info('Processing guide')
            response.success = True
            response.message = 'guide'
        elif request.command == "comeback":
            self.get_logger().info('Processing comeback')
            response.success = True
            response.message = 'comeback'
        else:
            self.get_logger().error('Unknown command received')
            response.success = False
            response.message = 'Unknown'

        # 서비스 응답 전송 후, user_command 클라이언트 호출
        if response.success:
            self.send_robot_command(request.command, request.description)
        
        return response
    

    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('User_command service not available')
            return
        if not self.robot_command_client_2.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('User_command service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.send_robot_command_callback)

        future = self.robot_command_client_2.call_async(request)
        future.add_done_callback(self.send_robot_command_callback_2)


    def send_robot_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot_command executed successfully (Robot to User): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute User_command (Robot to User) : %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))

    def send_robot_command_callback_2(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot_command executed successfully (Robot to Driving): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute User_command (Robot to Driving) : %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))
    
    

    def user_commnad_callback(self, request, response):
        self.get_logger().info('User Command Server started')
        if request.command == "touch_art":
            self.get_logger().info('Received touch_art')
            response.success = True
            response.message = 'touch_art'
        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'

        # 서비스 응답 전송 후, user_command 클라이언트 호출
        if response.success:
            self.send_user_command(request.command, request.description)
        
        return response


    # voice 끝난 후 .wav를 request.description에 담아서 Admin으로 보내기
    def send_user_command(self, command: str, description: str = ""):
        if not self.user_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('User_command service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.user_command_client.call_async(request)
        future.add_done_callback(self.send_user_command_callback)


    def send_user_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('User_command executed successfully (Robot to Admin): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute User_command (Robot to Admin): %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


def main(args=None):
    rp.init(args=args)

    robot_service = RobotService()

    rp.spin(robot_service)

    robot_service.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()