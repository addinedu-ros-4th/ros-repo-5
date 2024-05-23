import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String  # 명령을 위한 메시지 타입 변경
from ros_package_msgs.msg import RobotState
from ros_package_msgs.srv import CommandString
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
import threading
import time

class RobotDriver(Node):
    def __init__(self):
        super().__init__('Robot_Driver')

        # 로봇 상태 나타내는 퍼블리셔 
        self.robot_state_pose_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        # Robot_Driving/robot_command 토픽을 받아오는 구독자 생성
        self.robot_command_subscriber = self.create_subscription(
            String, 
            '/Robot_Driving/robot_command', 
            self.robot_command_callback, 
            10
        )

        # voice_recognize로 voice_start 서비스를 보냄
        self.voice_signal_client = self.create_client(CommandString, '/voice_signal')
        
        # 기본적으로 navigator를 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 순찰 상태 관리
        self.current_state = 'Patrolling'
        self.patrolling = True
        self.command_received = False

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 주기적으로 로봇 상태를 퍼블리시
        self.publish_robot_state()

        self.patrol_work()

    
    def publish_robot_state(self):
        msg = RobotState()
        msg.command = self.current_state

        self.robot_state_pose_publisher.publish(msg)
        self.get_logger().info('Published robot state: {}'.format(msg.command))


    def robot_command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received command: {}'.format(command))
        self.command_received = True
        if command == "human_detect":
            self.get_logger().info('Received human_detect')
            self.patrolling = False
            self.current_state = 'Human Detection'
            self.handle_human_detect()
        elif command == "description":
            self.get_logger().info('Received description')
            self.patrolling = False
            self.current_state = 'Description'
            self.handle_description()
        elif command == "guide":
            self.get_logger().info('Received guide')
            self.patrolling = False
            self.current_state = 'Guiding'
            self.handle_guide()
        elif command == "comeback":
            self.get_logger().info('Received comeback')
            self.current_state = 'Returning to Patrol'
            self.comeback_to_patrol()
        else:
            self.get_logger().error('Received Unknown command')
            self.command_received = False
    
    def handle_human_detect(self):
        self.send_voice_start_command('voice_start')
        
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

    def handle_description(self):
        self.get_logger().info('Robot stopped for description.')

    def handle_guide(self, description=""):
        self.get_logger().info('Robot guiding.')

    def comeback_to_patrol(self):
        self.send_voice_start_command('voice_stop') 
        self.patrolling = True
        self.patrol_work()

    def patrol_work(self):
        goal_poses = [
            ((0.8, 0.8, 0.0), (0.0, 0.0, 0.0)),  # 첫 번째 목표 지점
            ((1.3, 4.0, 0.0), (0.0, 0.0, 0.0)),  # 두 번째 목표 지점
            ((6.0, 3.7, 0.0), (0.0, 0.0, 0.0)),  # 세 번째 목표 지점
            ((6.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 네 번째 목표 지점
            ((10.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 다섯 번째 목표 지점
            ((10.0, 3.3, 0.0), (0.0, 0.0, 0.0)),  # 여섯 번째 목표 지점
            ((15.0, 2.8, 0.0), (0.0, 0.0, 0.0)),  # 일곱 번째 목표 지점
            ((15.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 여덟 번째 목표 지점
        ]

        while rp.ok():
            if self.patrolling and not self.command_received:
                for i in range(len(goal_poses)):
                    index = i if self.forward_patrol else len(goal_poses) - 1 - i
                    goal_pose, orientation = goal_poses[index]

                    goal_x, goal_y, goal_z = goal_pose
                    goal_orientation_x, goal_orientation_y, goal_orientation_z = orientation

                    goal_pose_msg = PoseStamped()
                    goal_pose_msg.header.frame_id = 'map'
                    goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
                    goal_pose_msg.pose.position.x = goal_x
                    goal_pose_msg.pose.position.y = goal_y
                    goal_pose_msg.pose.position.z = goal_z
                    goal_pose_msg.pose.orientation.x = goal_orientation_x
                    goal_pose_msg.pose.orientation.y = goal_orientation_y
                    goal_pose_msg.pose.orientation.z = goal_orientation_z
                    goal_pose_msg.pose.orientation.w = 1.0

                    self.get_logger().info('Navigating to goal: ({}, {}, {})'.format(goal_x, goal_y, goal_z))

                    # 목표 위치로 이동
                    self.navigator.goToPose(goal_pose_msg)

                    # 목표 지점에 도착하기를 기다림
                    while not self.navigator.isTaskComplete():
                        if self.command_received:
                            break

                    result = self.navigator.getResult()
                    
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
                        self.current_state = 'Stop at goal_pose'
                    elif result == TaskResult.CANCELED:
                        self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
                    elif result == TaskResult.FAILED:
                        self.get_logger().info('Goal failed at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
                    
                    # 순찰 방향 업데이트
                    if self.forward_patrol and index == len(goal_poses) - 1:
                        self.forward_patrol = False
                    elif not self.forward_patrol and index == 0:
                        self.forward_patrol = True

                    # 목표 지점 도착 후 3초 대기 후 커맨드가 있으면 처리
                    time.sleep(3)
                    if self.command_received:
                        break

def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
