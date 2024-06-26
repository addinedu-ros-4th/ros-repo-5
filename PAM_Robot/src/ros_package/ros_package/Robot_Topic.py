import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ros_package_msgs.msg import Voice
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseWithCovarianceStamped 
import time

class RobotTopic(Node):
    def __init__(self):
        super().__init__('Robot_Topic')

        # 카메라 토픽 구독자 생성 (웹캠 사용시 /camera)
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',
            self.camera_callback,
            10
        )

        # amcl_pose 토픽 구독자 생성 
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # robot_state 토픽 구독자 생성 
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # user_voice 토픽 구독자 생성 
        self.user_voice_subscription = self.create_subscription(
            Voice,
            '/recognized_text',
            self.user_voice_callback,
            10
        )

        # Admin_Manager로 퍼블리셔 생성
        self.admin_camera_publisher = self.create_publisher(CompressedImage, 'Admin_Manager/camera/compressed', 10)
        self.admin_amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'Admin_Manager/amcl_pose', 10)
        self.admin_robot_state_publisher = self.create_publisher(RobotState, 'Admin_Manager/robot_state', 10)
        self.admin_user_voice_publisher = self.create_publisher(Voice, 'Admin_Manager/recognized_text', 10)

        # User_GUI로 퍼블리셔 생성
        self.user_amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'User_GUI/amcl_pose', 10)

        # prevent unused variable warning
        self.camera_subscription
        self.pose_subscription
        self.robot_state_subscription
        self.user_voice_subscription


    def camera_callback(self, msg):
        # self.get_logger().info('Received /camera/image_raw , publishing to Admin_Manager/camera')

        self.admin_camera_publisher.publish(msg)


    def pose_callback(self, msg):
        # self.get_logger().info('Received /amcl_pose , publishing to Admin_Manager/amcl_pose , User_GUI/amcl_pose')

        self.admin_amcl_pose_publisher.publish(msg)
        self.user_amcl_pose_publisher.publish(msg)


    def robot_state_callback(self, msg):
        # self.get_logger().info('Received /robot_state , publishing to Admin_Manager/robot_state')

        self.admin_robot_state_publisher.publish(msg)

    def user_voice_callback(self, msg):
        print('voice')
        self.admin_user_voice_publisher.publish(msg)


def main(args=None):
    rp.init(args=args)

    robot_manager = RobotTopic()

    rp.spin(robot_manager)

    robot_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()