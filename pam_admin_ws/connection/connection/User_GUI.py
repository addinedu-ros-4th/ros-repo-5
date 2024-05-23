import rclpy as rp
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped  # /amcl_pose의 메세지 타입
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge  # cv_bridge 임포트

class UserGUI(Node):
    def __init__(self):
        super().__init__('User_GUI')

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        self.bridge = CvBridge()  # CvBridge 객체 초기화

        self.publisher = self.create_publisher(
                                    Image, 
                                    '/amcl_map', 
                                    10)
        
        # 주기적으로 위치를 업데이트하는 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # User_GUI/script 서비스를 받아오는 서비스 서버 생성
        self.script_server = self.create_service(SetBool, 'User_GUI/script', self.script_callback)

        # Admin_Manager로 /mic 서비스를 보내는 서비스 클라이언트 생성
        self.mic_client = self.create_client(SetBool, '/mic')

        # 이미지 읽기
        self.map_image = cv2.imread('/home/hj/amr_ws/ROS/src/my_mobile/PAM_world.pgm', cv2.IMREAD_GRAYSCALE)

        # 이미지 90도 반시계방향 회전
        self.map_image = cv2.rotate(self.map_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # 맵 정보 설정
        self.resolution = 0.05  # 맵의 해상도
        self.origin = [-2.6, -1.57, 0]  # 맵의 원점

        # 이전 로봇 위치
        self.prev_robot_pose = None

        # 초기 로봇 위치 설정
        self.robot_pose_x = int(-self.origin[0] / self.resolution)
        self.robot_pose_y = int(-self.origin[1] / self.resolution)

        # 맵 이미지를 화면에 표시
        self.update_map_image()

    def timer_callback(self):
        # 위치를 주기적으로 업데이트
        self.update_map_image()


    def pose_callback(self, msg):
        self.get_logger().info(f'x position: {msg.pose.pose.position.x}, y position: {msg.pose.pose.position.y}')

        # 로봇의 위치 받아오기
        self.robot_pose_x = int((-(msg.pose.pose.position.y) - self.origin[0]) / self.resolution)  # x 위치
        self.robot_pose_y = int((msg.pose.pose.position.x - self.origin[1]) / self.resolution)  # y 위치


    def update_map_image(self):
        # 맵 이미지의 높이
        map_height = self.map_image.shape[0]

        # 이전 로봇 위치 지우기
        if self.prev_robot_pose is not None:
            cv2.circle(self.map_image, self.prev_robot_pose, 3, (255, 255, 255), -1)

        # 로봇 위치를 이미지 좌표로 변환 (상하 반전)
        robot_image_y = map_height - self.robot_pose_y

        # 맵 이미지에 새로운 로봇 위치 표시
        cv2.circle(self.map_image, (self.robot_pose_x, robot_image_y), 3, (0, 255, 0), -1)

        # 이전 로봇 위치 업데이트
        self.prev_robot_pose = (self.robot_pose_x, robot_image_y)

        img_msg = self.bridge.cv2_to_imgmsg(self.map_image, encoding="mono8")

        self.publisher.publish(img_msg)




    def script_callback(self, request, response):
        self.get_logger().info('Received script')
        
        # 받은 스크립트를 GUI에 띄우기!
        self.script = request.data

        print(self.script)
        
        response.success = True
        response.message = 'Script received and sent to User_GUI'
        
        return response


    def mic_callback(self, request, response):
        if request.data:
            self.get_logger().info('Received mic data, forwarding to Robot_Manager')
            # Robot_Manager /mic 서비스 요청 보내기
            self.send_mic_to_robot_manager(request.data)
            
            response.success = True
            response.message = 'Mic data received and forwarded to Robot_Manager'
        else:
            self.get_logger().info('Received mic data, but it is empty')
            response.success = False
            response.message = 'Mic data is empty'

        return response


    def send_mic_to_robot_manager(self, mic_data):
        while not self.mic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot_Manager/mic service not available, waiting...')

        request = SetBool.Request()
        request.data = mic_data
        future = self.mic_client.call_async(request)
        future.add_done_callback(self.send_mic_callback)


    def send_mic_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Mic data successfully sent to Robot_Manager')
            else:
                self.get_logger().error('Failed to send mic data to Robot_Manager: %s' % response.message)
        except Exception as e:
            self.get_logger().error('Failed to send mic data to Robot_Manager: %s' % str(e))


def main(args=None):
    rp.init(args=args)

    robot_manager = UserGUI()
    rp.spin(robot_manager)

    robot_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
