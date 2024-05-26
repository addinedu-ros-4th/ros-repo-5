import rclpy as rp
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
import threading
import time , asyncio
from tf_transformations import quaternion_from_euler
import math

class RobotDriver(Node):
    def __init__(self):
        super().__init__('Robot_Driver')

        # 로봇 상태 나타내는 퍼블리셔 
        self.robot_state_pose_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        # Robot_Driving/robot_command 서비스를 받아오는 서비스 서버 생성
        self.robot_command_server = self.create_service(CommandString, '/Robot_Driving/robot_command', self.robot_command_callback)
        
        # 기본적으로 navigator를 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 순찰 상태 관리 Patrolling
        self.current_state = 'arrive'
        self.patrolling = True
        self.command_received = False

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 주기적으로 로봇 상태를 퍼블리시
        self.state_check_timer = self.create_timer(1.0, self.publish_robot_state)

        # 순찰 경로 정의
        self.route_forward = [
            (2.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(0))),
            (6.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (2.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (2.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(180))),
            (0.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
        ]

        self.route_reverse = [
            (6.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (2.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (2.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(180))),
            (2.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
        ]
        
        self.description_point = [
            (4.0, -1.5, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (4.0, -4.25, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (3.0, -8.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (3.0, -10.5, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (4.0, -14.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (4.0, -16.0, 0.0, *self.euler_to_quaternion(math.radians(180))),
        ]
    
        self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        self.patrol_work()

    def euler_to_quaternion(self, yaw, pitch=0.0, roll=0.0):
        return quaternion_from_euler(roll, pitch, yaw)
    
    def set_goal_pose(self, x, y, z, qx, qy, qz, qw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        return goal_pose
    
    def set_initial_pose(self, x, y, z, qx, qy, qz, qw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = z
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)

    def patrol_work(self):
        routes = [self.route_forward, self.route_reverse]
        current_route_index = 0

        while rp.ok():
            if self.patrolling and not self.command_received:
                current_route = routes[current_route_index]
                for i, pt in enumerate(current_route):
                    if self.command_received:
                        break
                    goal_pose = self.set_goal_pose(*pt)
                    self.navigator.goToPose(goal_pose)
                    while not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        if feedback:
                            self.get_logger().info('Distance remaining: {:.2f}'.format(feedback.distance_remaining))

                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info('Reached goal: ({}, {})'.format(pt[0], pt[1]))

                        # human_detect 상태 확인 및 description_point로 이동
                        if self.current_state == 'Human Detection':
                            description_pose = self.get_description_pose(current_route_index, i)
                            self.navigator.goToPose(description_pose)
                            while not self.navigator.isTaskComplete():
                                pass  # 대기

                            self.get_logger().info('Reached description point')
                            self.current_state = 'arrive'
                            self.command_received = False  # Reset command_received 상태
                            self.patrolling = True  # 다시 순찰 시작
                    elif result == TaskResult.CANCELED:
                        self.get_logger().info('Goal was canceled, exiting patrol.')
                        return
                    elif result == TaskResult.FAILED:
                        self.get_logger().info('Failed to reach goal: ({}, {}), retrying...'.format(pt[0], pt[1]))
                        break

                current_route_index = (current_route_index + 1) % 2

    def get_description_pose(self, route_index, goal_index):
        if route_index == 0:  # 정방향
            mapping = {1: 0, 3: 2, 4: 1, 5: 4, 6: 3, 8: 5}
        else:  # 역방향
            mapping = {2: 3, 3: 4, 4: 1, 5: 2, 7: 0}
        
        description_index = mapping.get(goal_index)
        if description_index is not None:
            description_pose = self.set_goal_pose(*self.description_point[description_index])
            return description_pose
        return None

    def publish_robot_state(self):
        msg = RobotState()
        msg.command = self.current_state
        self.robot_state_pose_publisher.publish(msg)
        
    def robot_command_callback(self, request, response):
        self.get_logger().info('Robot Command Server started')

        self.command_received = True
        
        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'

            self.patrolling = False
            self.current_state = 'Human Detection'

        elif request.command == "description":
            self.get_logger().info('Received description')
            response.success = True
            response.message = 'description'

            self.patrolling = False
            self.current_state = 'Description'

            self.handle_description()
            

        elif request.command == "guide":
            self.get_logger