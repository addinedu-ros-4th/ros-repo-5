import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped # /amcl_pose의 메세지 타입
from cv_bridge import CvBridge
import cv2

class Admin_Manager(Node):
    def __init__(self):
        super().__init__('User_GUI')

        # 카메라 토픽 구독자 생성
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


        # OpenCV 이미지와 ROS 이미지 간 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()

        # 맵 이미지 로드
        self.map_image = cv2.imread('/home/hj/amr_ws/ROS/src/my_mobile/PAM_world.pgm', cv2.IMREAD_GRAYSCALE)

        # 맵 정보 설정
        self.resolution = 0.05  # 맵의 해상도
        self.origin = [-1.12, -2.57, 0]  # 맵의 원점

        # 이전 로봇 위치
        self.prev_robot_pose = None

        # 초기 로봇 위치 설정
        self.robot_pose_x = int(-self.origin[0] / self.resolution)
        self.robot_pose_y = int(-self.origin[1] / self.resolution)

        # 맵 이미지를 화면에 표시
        self.update_map_image()

        # 주기적으로 위치를 업데이트하는 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)


    def camera_callback(self, msg):
        self.get_logger().info('Received /camera')

        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # 이미지를 화면에 표시
            cv2.imshow("Camera", cv_image)
            cv2.waitKey(1)  # 이미지가 표시되도록 윈도우를 업데이트합니다.
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def timer_callback(self):
        # 위치를 주기적으로 업데이트
        self.update_map_image()


    def pose_callback(self, msg):
        self.get_logger().info('Received /amcl_pose')

        # 로봇의 위치 받아오기
        self.robot_pose_x = int((msg.pose.pose.position.x - self.origin[0]) / self.resolution)  # x 위치
        self.robot_pose_y = int((msg.pose.pose.position.y - self.origin[1]) / self.resolution)  # y 위치


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

        # 맵 이미지를 화면에 표시
        cv2.imshow('Map with Robot Pose', self.map_image)
        cv2.waitKey(1)



def main(args=None):
    rp.init(args=args)

    robot_manager = Admin_Manager()
    rp.spin(robot_manager)

    robot_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
