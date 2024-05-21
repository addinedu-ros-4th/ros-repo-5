import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros_package_msgs.srv import CommandString 
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseWithCovarianceStamped 
# from cv_bridge import CvBridge
# import cv2

class Admin_Manager(Node):
    def __init__(self):
        super().__init__('User_GUI')

        # camera 토픽 구독자 생성
        self.camera_subscription = self.create_subscription(
            Image,
            'Admin_Manager/camera',
            self.camera_callback,
            10
        )

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'Admin_Manager/amcl_pose',
            self.pose_callback,
            10
        )

        # robot_state 토픽 구독자 생성 
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            'Admin_Manager/robot_state',
            self.robot_state_callback,
            10
        )

        # Admin_Manager/user_command 서비스 받기 
        self.user_command_server = self.create_service(CommandString, 'Admin_Manager/user_command', self.user_command_callback)

        #  /robot_command 서비스 보내기 
        self.robot_command_client = self.create_client(CommandString, '/robot_command')

        # # OpenCV 이미지와 ROS 이미지 간 변환을 위한 CvBridge 객체 생성
        # self.bridge = CvBridge()

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


    def camera_callback(self, msg):
        self.get_logger().info('Received /camera')

        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #     cv2.imshow("Camera", cv_image)
        #     cv2.waitKey(1) 

        # except Exception as e:
        #     self.get_logger().error(f"Error processing image: {e}")


    def pose_callback(self, msg):
        self.get_logger().info('Received /amcl_pose')

        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution) 


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


    def robot_state_callback(self, msg):
        self.get_logger().info('Received /robot_state')
        robot_state = msg

    def user_command_callback(self, request, response):
        self.get_logger().info('User_Command Server started')
        if request.command == "voice":
            self.get_logger().info('Received voice')
            response.success = True
            response.message = 'voice'
            print(request.command , request.description)
        elif request.command == "touch_art":
            self.get_logger().info('Received touch_art')
            response.success = True
            response.message = 'touch_art'
            print(request.command , request.description)
        else:
            self.get_logger().error('Received Unknown')
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
                self.get_logger().info('Robot command executed successfully (Admin to Robot): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute robot command (Admin to Robot): %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


def main(args=None):
    rp.init(args=args)

    admin_manager = Admin_Manager()

    rp.spin(admin_manager)

    # admin_manager.send_robot_command("human_detect")
    # admin_manager.send_robot_command("description", "작품설명...")
    # admin_manager.send_robot_command("guide")
    # admin_manager.send_robot_command("comeback")
    # admin_manager.send_robot_command("e")

    admin_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()