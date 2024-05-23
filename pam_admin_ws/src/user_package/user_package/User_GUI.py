import rclpy as rp
from rclpy.node import Node
from ros_package_msgs.srv import CommandString
from geometry_msgs.msg import PoseWithCovarianceStamped  
# from sensor_msgs.msg import Image
# import cv2

class UserGUI(Node):
    def __init__(self):
        super().__init__('User_GUI')

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'User_GUI/amcl_pose',
            self.pose_callback,
            10
        )

        # User_GUI/robot_command 서비스를 받아오는 서비스 서버 생성
        self.robot_command_server = self.create_service(CommandString, 'User_GUI/robot_command', self.robot_command_callback)

        # Admin_Manager로 /user_command 서비스를 보내는 서비스 클라이언트 생성
        self.user_command_client = self.create_client(CommandString, '/user_command')

        # # 맵 이미지 로드
        # self.map_image = cv2.imread('/home/jongchanjang/my_mobile/MUSEUM.pgm', cv2.IMREAD_GRAYSCALE)

        # # 맵 정보 설정
        # self.resolution = 0.05  # 맵의 해상도
        # self.origin = [-1.12, -2.57, 0]  # 맵의 원점

        # # 이전 로봇 위치
        # self.prev_robot_pose = None

        # # 초기 로봇 위치 설정
        # self.robot_pose_x = int(-self.origin[0] / self.resolution)
        # self.robot_pose_y = int(-self.origin[1] / self.resolution)

        # # 맵 이미지를 화면에 표시
        # self.update_map_image()

        # # 주기적으로 위치를 업데이트하는 타이머 설정
        # self.timer = self.create_timer(1.0, self.timer_callback)


    def pose_callback(self, msg):
        self.get_logger().info('Received /amcl_pose')

        # 로봇의 위치 받아오기
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  # x 위치
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)  # y 위치


    # def update_map_image(self):
    #     # 맵 이미지의 높이
    #     map_height = self.map_image.shape[0]

    #     # 이전 로봇 위치 지우기
    #     if self.prev_robot_pose is not None:
    #         cv2.circle(self.map_image, self.prev_robot_pose, 3, (255, 255, 255), -1)

    #     # 로봇 위치를 이미지 좌표로 변환 (상하 반전)
    #     robot_image_y = map_height - self.robot_pose_y

    #     # 맵 이미지에 새로운 로봇 위치 표시
    #     cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 3, (0, 255, 0), -1)

    #     # 이전 로봇 위치 업데이트
    #     self.prev_robot_pose = (self.robot_pose_x, robot_image_y)

    #     # 맵 이미지를 화면에 표시
    #     cv2.imshow('Map with Robot Pose', self.map_image)
    #     cv2.waitKey(1)


    # def timer_callback(self):
    #     # 위치를 주기적으로 업데이트
    #     self.update_map_image()


    def robot_command_callback(self, request, response):
        self.get_logger().info('Robot Command Server started')
        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'
        elif request.command == "description":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'description'
            print(request.description)
        elif request.command == "guide":
            self.get_logger().info('Received guide')
            response.success = True
            response.message = 'guide'
        elif request.command == "comeback":
            self.get_logger().info('Received comeback')
            response.success = True
            response.message = 'comeback'
        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'

        return response


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
                self.get_logger().info('User command executed successfully (User to Robot): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute robot command (User to Robot): %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


def main(args=None):
    rp.init(args=args)

    user_gui = UserGUI()

    rp.spin(user_gui)

    # user_gui.send_user_command('touch_art','피라미드')
    # user_gui.send_user_command('e')

    user_gui.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()