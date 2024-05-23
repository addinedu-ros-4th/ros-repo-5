import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # /map 토픽의 메시지 타입
from sensor_msgs.msg import Image  # /camera 토픽의 메시지 타입
from std_srvs.srv import SetBool # /mic 서비스의 메세지 타입
from geometry_msgs.msg import PoseWithCovarianceStamped # /amcl_pose의 메세지 타입
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult


class RobotManager(Node):
    def __init__(self):
        super().__init__('Robot_Manager')

        # 카메라 토픽 구독자 생성 (웹캠 사용시 /camera)
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
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

        # Admin_Manager로 퍼블리셔 생성
        self.admin_camera_publisher = self.create_publisher(Image, 'Admin_Manager/camera', 10)
        self.admin_amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'Admin_Manager/amcl_pose', 10)

        # User_GUI로 퍼블리셔 생성 
        self.user_amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'User_GUI/amcl_pose', 10)

        # prevent unused variable warning
        self.camera_subscription

        # Admin_Manager로 robot_command 서비스를 받아옴
        self.robot_command_server = self.create_service(SetBool, 'robot_command', self.command_callback)

        # Admin_Manager로 script 서비스를 받아옴
        self.script_server = self.create_service(SetBool, 'script', self.script_callback)

        # User_GUI로 User_GUI/script 서비스를 보내는 서비스 클라이언트 생성
        self.script_client = self.create_client(SetBool, 'User_GUI/script')

        # /mic 서비스를 받아오는 서비스 서버 생성
        self.mic_server = self.create_service(SetBool, 'mic', self.mic_callback)

        # Admin_Manager로 /mic 서비스를 보내는 서비스 클라이언트 생성
        self.mic_client = self.create_client(SetBool, 'Admin_Manager/mic')

        # User_GUI로 user_command 서비스를 받아옴
        self.user_command_server = self.create_service(SetBool, 'user_command', self.user_callback)

    # 테스트 중 (주행패키지 입력)
    # def robot_work(self):
    #     rp.init()
    #     nav = BasicNavigator()
    #     nav.waitUntilNav2Active()

    #     # 목표 지점과 초기 위치를 정의합니다.
    #     goal_poses = [
    #         ((0.8, 0.8, 0.0), (0.0, 0.0, 0.0)),  # 첫 번째 목표 지점
    #         ((1.3, 4.0, 0.0), (0.0, 0.0, 0.0)),  # 두 번째 목표 지점
    #         ((6.0, 3.7, 0.0), (0.0, 0.0, 0.0)),  # 세 번째 목표 지점
    #         ((6.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 네 번째 목표 지점
    #         ((10.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 다섯 번째 목표 지점
    #         ((10.0, 3.3, 0.0), (0.0, 0.0, 0.0)),  # 여섯 번째 목표 지점
    #         ((15.0, 2.8, 0.0), (0.0, 0.0, 0.0)),  # 일곱 번째 목표 지점
    #         ((15.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 여덟 번째 목표 지점
    #     ]

    #     for goal_pose, orientation in goal_poses:
    #         goal_x, goal_y, goal_z = goal_pose
    #         goal_orientation_x, goal_orientation_y, goal_orientation_z = orientation

    #         goal_pose_msg = PoseStamped()
    #         goal_pose_msg.header.frame_id = 'map'
    #         goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
    #         goal_pose_msg.pose.position.x = goal_x
    #         goal_pose_msg.pose.position.y = goal_y
    #         goal_pose_msg.pose.position.z = goal_z
    #         goal_pose_msg.pose.orientation.x = goal_orientation_x
    #         goal_pose_msg.pose.orientation.y = goal_orientation_y
    #         goal_pose_msg.pose.orientation.z = goal_orientation_z
    #         goal_pose_msg.pose.orientation.w = 1.0

    #         nav.goToPose(goal_pose_msg)

    #         while not nav.isTaskComplete():
    #             # 이동이 완료될 때까지 대기합니다.
    #             pass

    #         result = nav.getResult()
    #         if result == TaskResult.SUCCEEDED:
    #             self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
    #         elif result == TaskResult.CANCELED:
    #             self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
    #         elif result == TaskResult.FAILED:
    #             self.get_logger().info('Goal failed at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

    #     nav.shutdown()
    #     rp.shutdown()


    ### 카메라 토픽 받고 보내는거 확인완료 ### 
    def camera_callback(self, msg):
        # self.get_logger().info('Received /camera/image_raw , publishing to Admin_Manager/camera')

        # Admin_Manager로 퍼블리시
        self.admin_camera_publisher.publish(msg)


    ### amcl_pose 토픽 받고 보내기 확인 ### 
    def pose_callback(self, msg):
        # self.get_logger().info('Received /amcl_pose , publishing to Admin_Manager/amcl_pose , User_GUI/amcl_pose')

        # Admin_Manager , User_GUI 로 퍼블리시
        self.admin_amcl_pose_publisher.publish(msg)
        self.user_amcl_pose_publisher.publish(msg)


    # request 종류에 따라 행동을 다르게!
    def command_callback(self, request, response):
        self.get_logger().info('Robot Command Server started')
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
    

    # User_GUI에서 종료 버튼을 누르면 경로복귀하는 함수 
    def user_callback(self, request, response):
        self.get_logger().info('User Command Server started')
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
    

    def script_callback(self, request, response):
        self.get_logger().info('Received script')
        
        # 받은 스크립트를 User_GUI로 보내기
        self.send_script_to_user_gui(request.data)
        
        response.success = True
        response.message = 'Script received and sent to User_GUI'
        
        return response
    

    def send_script_to_user_gui(self, script):
        # 서비스 요청을 보내기 전에 서비스가 활성화되었는지 확인
        while not self.script_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = script
        future = self.script_client.call_async(request)
        future.add_done_callback(self.send_script_callback)


    def send_script_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Script successfully sent  to User_GUI')
            else:
                self.get_logger().error('Failed to send script to User_GUI')
        except Exception as e:
            self.get_logger().error('Failed to send script to User_GUI: %s' % str(e))
    

    def mic_callback(self, request, response):
        if request.data:
            self.get_logger().info('Received mic data, forwarding to Admin_Manager')
            # Admin_Manager로 /mic 서비스 요청 보내기
            self.send_mic_to_admin_manager(request.data)
            
            response.success = True
            response.message = 'Mic data received and forwarded to Admin_Manager'
        else:
            self.get_logger().info('Received mic data, but it is empty')
            response.success = False
            response.message = 'Mic data is empty'

        return response


    def send_mic_to_admin_manager(self, mic_data):
        while not self.mic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Admin_Manager/mic service not available, waiting...')

        request = SetBool.Request()
        request.data = mic_data
        future = self.mic_client.call_async(request)
        future.add_done_callback(self.send_mic_callback)


    def send_mic_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Mic data successfully sent to Admin_Manager')
            else:
                self.get_logger().error('Failed to send mic data to Admin_Manager: %s' % response.message)
        except Exception as e:
            self.get_logger().error('Failed to send mic data to Admin_Manager: %s' % str(e))

    
def main(args=None):
    rp.init(args=args)

    robot_manager = RobotManager()
    rp.spin(robot_manager)

    robot_manager.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
