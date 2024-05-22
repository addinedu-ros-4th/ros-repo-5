import rclpy as rp
from rclpy.node import Node
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
import threading
import math

class RobotDriver(Node):
    def __init__(self):
        super().__init__('Robot_Driver')

        # 로봇 상태 나타내는 퍼블리셔 
        self.robot_state_pose_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        # Robot_Driving/robot_command 서비스를 받아오는 서비스 서버 생성
        self.robot_command_server = self.create_service(CommandString, '/Robot_Driving/robot_command', self.robot_command_callback)

        # voice_recognize로 voice_start 서비스를 보냄
        self.voice_signal_client = self.create_client(CommandString, '/voice_signal')
        
        # # 기본적으로 navigator를 초기화
        # self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active()

        # # 순찰 스레드 시작
        # self.patrol_thread = threading.Thread(target=self.patrol_work)
        # self.patrol_thread.start()

        # 순찰 상태 관리
        self.current_state = 'Patrolling'
        self.patrolling = True
        self.command_received = False

        # 주기적으로 로봇 상태를 퍼블리시하는 타이머 설정 (1초 간격)
        self.create_timer(1.0, self.publish_robot_state)


    def publish_robot_state(self):
        # 예제 메시지 생성
        msg = RobotState()
        msg.command = self.current_state

        self.robot_state_pose_publisher.publish(msg)
        # self.get_logger().info('Published robot state: {}'.format(msg.command))


    def robot_command_callback(self, request, response):
        self.get_logger().info('Robot Command Server started')
        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'
            self.command_received = True
            self.patrolling = False
            self.current_state = 'Human Detection'
            # self.handle_human_detect()
        elif request.command == "description":
            self.get_logger().info('Received description')
            response.success = True
            response.message = 'description'
            self.command_received = True
            self.patrolling = False
            self.current_state = 'Description'
            # self.handle_description()
        elif request.command == "guide":
            self.get_logger().info('Received guide')
            response.success = True
            response.message = 'guide'
            self.command_received = True
            self.patrolling = False
            self.current_state = 'Guiding'
            # self.handle_guide(request.description)
        elif request.command == "comeback":
            self.get_logger().info('Received comeback')
            response.success = True
            response.message = 'comeback'
            self.command_received = True
            self.patrolling = False
            self.current_state = 'Returning to Patrol'
            # self.comeback_to_patrol()
        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'

        return response
    

    # def handle_human_detect(self):
    #     target_pose = PoseStamped()
    #     target_pose.header.frame_id = 'map'
    #     target_pose.header.stamp = self.get_clock().now().to_msg()
    #     target_pose.pose.position.x = 1.0  
    #     target_pose.pose.position.y = 1.0  
    #     target_pose.pose.orientation.w = 1.0

    #     self.navigate_to_goal(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z)
        # self.send_voice_start_command('voice_start') # or voice_stop


    # def handle_description(self):
    #     self.navigator.cancelTask()
    #     self.get_logger().info('Robot stopped for description.')

        
    def send_voice_start_command(self, command: str, description: str = ""):
        if not self.voice_signal_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Voice_siganl service not available')
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
                self.get_logger().info('Voice_siganl executed successfully (Driving to voice): %s' % response.message)
            else:
                self.get_logger().error('Failed to execute Voice_siganl (Driving to voice): %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


    # def handle_guide(self, description):
    #     x, y = map(float, description.split(','))
    #     self.navigate_to_goal(x, y, 0.0)


    # def comeback_to_patrol(self):
    #     # 현재 위치 가져오기
    #     current_pose = self.navigator.getCurrentPose().pose

    #     # 순찰 웨이포인트 정의
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

    #     # 가장 가까운 웨이포인트 계산
    #     closest_goal = None
    #     min_distance = float('inf')
    #     for goal_pose, _ in goal_poses:
    #         distance = math.sqrt((goal_pose[0] - current_pose.position.x) ** 2 + (goal_pose[1] - current_pose.position.y) ** 2)
    #         if distance < min_distance:
    #             min_distance = distance
    #             closest_goal = goal_pose

    #     # 가장 가까운 웨이포인트로 이동
    #     if closest_goal:
    #         self.navigate_to_goal(*closest_goal)
        
    #     # 순찰 재개
    #     self.patrolling = True


    # def navigate_to_goal(self, x, y, z):
    #     goal_pose_msg = PoseStamped()
    #     goal_pose_msg.header.frame_id = 'map'
    #     goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
    #     goal_pose_msg.pose.position.x = x
    #     goal_pose_msg.pose.position.y = y
    #     goal_pose_msg.pose.position.z = z
    #     goal_pose_msg.pose.orientation.w = 1.0

    #     self.navigator.goToPose(goal_pose_msg)

    #     while not self.navigator.isTaskComplete():
    #         # 이동이 완료될 때까지 대기합니다.
    #         pass

    #     result = self.navigator.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(x, y, z))
    #     elif result == TaskResult.CANCELED:
    #         self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(x, y, z))
    #     elif result == TaskResult.FAILED:
    #         self.get_logger().info('Goal failed at ({}, {}, {})!'.format(x, y, z))

    #     self.command_received = False  # 명령 처리 완료
    #     self.patrolling = True  # 순찰 재개


    # def patrol_work(self):
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

    #     while rp.ok():
    #         for goal_pose, orientation in goal_poses:
    #             if not self.patrolling or self.command_received:
    #                 continue

    #             goal_x, goal_y, goal_z = goal_pose
    #             goal_orientation_x, goal_orientation_y, goal_orientation_z = orientation

    #             goal_pose_msg = PoseStamped()
    #             goal_pose_msg.header.frame_id = 'map'
    #             goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
    #             goal_pose_msg.pose.position.x = goal_x
    #             goal_pose_msg.pose.position.y = goal_y
    #             goal_pose_msg.pose.position.z = goal_z
    #             goal_pose_msg.pose.orientation.x = goal_orientation_x
    #             goal_pose_msg.pose.orientation.y = goal_orientation_y
    #             goal_pose_msg.pose.orientation.z = goal_orientation_z
    #             goal_pose_msg.pose.orientation.w = 1.0

    #             self.navigator.goToPose(goal_pose_msg)

    #             while not self.navigator.isTaskComplete():
    #                 if self.command_received:
    #                     break

    #             result = self.navigator.getResult()
    #             if result == TaskResult.SUCCEEDED:
    #                 self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
    #                 self.current_state = 'Stop at goal_pose'
    #             elif result == TaskResult.CANCELED:
    #                 self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
    #             elif result == TaskResult.FAILED:
    #                 self.get_logger().info('Goal failed at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

    #             if self.command_received:
    #                 self.command_received = False

    # def shutdown(self):
    #     self.navigator.shutdown()


def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
