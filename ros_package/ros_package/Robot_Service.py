import rclpy as rp
from rclpy.node import Node
from ros_package_msgs.srv import CommandString
import time 

class RobotService(Node):
    def __init__(self):
        super().__init__('Robot_Service')

        # Admin_Manager로 robot_command 서비스를 받아옴
        self.robot_command_server = self.create_service(CommandString, '/robot_command', self.robot_command_callback)

        # Robot_Driving로 robot_command 서비스를 보냄
        self.robot_command_client = self.create_client(CommandString, 'Robot_Driving/robot_command')

        # User_GUI로 robot_command 서비스를 보냄
        self.robot_command_client_2 = self.create_client(CommandString, 'User_GUI/robot_command')

        # voice_recognize로 voice_start 서비스를 보냄
        self.voice_signal_client = self.create_client(CommandString, '/voice_signal')


    def robot_command_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        # 명령에 따라 응답 처리
        if request.command == "human_detect":
            self.get_logger().info('Processing human_detect')
            response.success = True
            response.message = 'human_detect'

            self.send_robot_command(request.command, request.description) # Robot_Driving : 작품설명 위치이동
            self.send_robot_command_2(request.command, request.description) # GUI음성 : 무엇을 도와드릴까요?
            self.send_voice_start_command('voice_start') #voice_recog : 음성열기

        elif request.command == "description":
            self.get_logger().info('Processing description')
            response.success = True
            response.message = 'description'

            self.send_voice_start_command('voice_stop') #voice_recog : 음성닫기
            self.send_robot_command(request.command, request.description) # Robot_Driving : 제자리 위치
            self.send_robot_command_2(request.command, request.description) # GUI음성 : 작품설명.... , 또다른 도움이 필요하십니까?
            self.send_voice_start_command('voice_start') #voice_recog : 음성열기

        elif request.command == "guide":
            self.get_logger().info('Processing guide')
            response.success = True
            response.message = 'guide'

            self.send_voice_start_command('voice_stop') #voice_recog : 음성닫기
            self.send_robot_command_2(request.command, request.description) # GUI음성 : ~로 길안내 해드리겠습니다.
            self.send_robot_command(request.command, request.description) # Robot_Driving : 작품으로 길안내
            self.send_robot_command_2(request.command, request.description) # GUI음성 : 또다른 도움이 필요하십니까?
            self.send_voice_start_command('voice_start') #voice_recog : 음성열기

        elif request.command == "comeback":
            self.get_logger().info('Processing comeback')
            response.success = True
            response.message = 'comeback'

            self.send_voice_start_command('voice_stop') #voice_recog : 음성닫기
            self.send_robot_command_2(request.command, request.description) # GUI음성 : 편안한 관람되십시오
            self.send_robot_command(request.command, request.description) # Robot_Driving : 작품설명 위치이동

        else:
            self.get_logger().error('Unknown command received')
            response.success = False
            response.message = 'Unknown'
            
        return response
    

    def send_robot_command(self, command: str, description: str = ""):
        if not self.robot_command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Robot_command service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.robot_command_client.call_async(request)
        future.add_done_callback(self.send_robot_command_callback)
        

    def send_robot_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot_command executed successfully (Robot to Driving): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute User_command (Robot to Driving) : %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


    def send_robot_command_2(self, command: str, description: str = ""):
        if not self.robot_command_client_2.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Robot_command service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.robot_command_client_2.call_async(request)
        future.add_done_callback(self.send_robot_command_callback_2)


    def send_robot_command_callback_2(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot_command executed successfully (Robot to User): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute User_command (Robot to User) : %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


    def send_voice_start_command(self, command: str, description: str = ""):
        if not self.voice_signal_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Voice_signal service not available')
            return

        request = CommandString.Request()
        request.command = command
        request.description = description

        future = self.voice_signal_client.call_async(request)
        future.add_done_callback(self.send_voice_start_command_callback)


    def send_voice_start_command_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Voice_signal executed successfully (Driving to voice): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute Voice_signal (Driving to voice): %s' % response.message)
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