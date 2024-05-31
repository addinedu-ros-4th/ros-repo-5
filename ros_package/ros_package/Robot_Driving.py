import rclpy as rp
from rclpy.node import Node
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
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
        
        # Robot_Driving/robot_command 서비스를 받아오는 서비스 서버 생성
        self.robot_command_server = self.create_service(CommandString, '/Robot_Driving/robot_command', self.robot_command_callback)

        # /patrol_command 서비스를 받아오는 서비스 서버 생성
        self.patrol_command_server = self.create_service(CommandString, '/patrol_command', self.patrol_command_callback)

        # # 기본적으로 navigator를 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 순찰 상태 관리
        self.current_state = 'Start'
        self.command_received = False

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 목표 지점의 인덱스 초기화
        self.index = 0

        # 웨이포인트 , 작품설명위치
        self.goal_poses = [
            ((0.00426, -0.143, -0.00143), (0.0, 0.0, 0.0)),  # 첫 번째 목표 지점
            ((1, -0.143, -0.00137), (0.0, 0.0, 0.0)),  # 두 번째 목표 지점
            ((1, -1.15, -0.00143), (0.0, 0.0, 0.0)),  # 세 번째 목표 지점
            ((0.2, -1.15, -0.00143), (0.0, 0.0, 0.0)),  # 네 번째 목표 지점
            ((0.2, -2.23, -0.00143), (0.0, 0.0, 0.0)),  # 다섯 번째 목표 지점
            ((1, -2,23, -0.00143), (0.0, 0.0, 0.0)),  # 여섯 번째 목표 지점
        ]

        self.goal_poses_art = ['강아지','2','고양이','초록양','5','갈색말'] # 작품위치에 따른 작품명
        self.current_art = None

        self.point = [
            ((2.2,1.6,0.0),(0.0,0.0,0.0,0.0)), # 첫번째 목표 지점의 작품설명위치
            ((0.0,0.0,0.0),(0.0,0.0,0.0,0.0)), # 두번째 목표 지점의 작품설명위치(작품없음)
            ((7.5,2.0,0.0),(0.0,0.0,0.0,0.0)), # 세번째 목표 지점의 작품설명위치
            ((4.6,1.8,0.0),(0.0,0.0,0.0,0.0)), # 네번째 목표 지점의 작품설명위치
            ((0.0,0.0,0.0),(0.0,0.0,0.0,0.0)), # 두번째 목표 지점의 작품설명위치(작품없음)
            ((9.0,1.2,0.0),(0.0,0.0,0.0,0.0)), # 여섯번째 목표 지점의 작품설명위치
        ]
        
        # 스타트 신호
        self.publish_robot_state()

        # 1번 웨이포인트로 이동
        self.patrol_work()

        # 멈춤상태 신호
        self.publish_robot_state()


    def publish_robot_state(self):
        msg = RobotState()
        msg.command = self.current_state

        self.robot_state_pose_publisher.publish(msg)


    def patrol_work(self):
        self.current_state = 'Patrolling'

        self.publish_robot_state()

        # 인덱스가 리스트 범위를 벗어나지 않도록 체크
        if self.index < 0:
            self.index = 0
            self.forward_patrol = True
        elif self.index >= len(self.goal_poses):
            self.index = len(self.goal_poses) - 1
            self.forward_patrol = False

        goal_pose, orientation = self.goal_poses[self.index]

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

        start_time = time.time()
        timeout = 20  # 20초 타임아웃

        while not self.navigator.isTaskComplete():
            if time.time() - start_time > timeout:
                self.navigator.cancelTask()
                self.get_logger().warn('Navigation to goal timed out!')
                break
            time.sleep(0.1)

        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))


        if self.index in [1, 4]:
            self.current_state = 'not item at {}'.format(self.goal_poses_art[self.index])
        else:
            self.current_state = 'arrive at {}'.format(self.goal_poses_art[self.index])

        self.current_art = self.goal_poses_art[self.index]


        # 현재 목표 지점 인덱스 업데이트
        if self.forward_patrol:
            self.index += 1
        else:
            self.index -= 1


    def patrol_command_callback(self, request, response):

        self.get_logger().info('Patrol Command Server started')

        if request.command == "patrol":
            self.get_logger().info('Received patrol')
            response.success = True
            response.message = 'patrol'   

            self.current_state = 'Patrolling'

            self.publish_robot_state()
        
            self.patrol_work()

            self.publish_robot_state()

        return response


    def robot_command_callback(self, request, response):
        
        self.get_logger().info('Robot Command Server started')

        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'   

            self.current_state = 'Human Detection'

            self.publish_robot_state()

            self.handle_human_detect()


        elif request.command == "description":
            self.get_logger().info('Received description')
            response.success = True
            response.message = 'description'

            self.current_state = 'Description'

            self.publish_robot_state()


        elif request.command == "guide":
            self.get_logger().info('Received guide')
            response.success = True
            response.message = 'guide'

            self.current_state = 'Guiding'

            self.publish_robot_state()

            self.handle_guide(request.description)

        elif request.command == "comeback":
            self.get_logger().info('Received comeback')
            response.success = True
            response.message = 'comeback'

            self.current_state = 'comeback_to_patrol'

            self.publish_robot_state()

            self.comeback_to_patrol()

            self.patrol_work()

            self.publish_robot_state()

        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'


        return response
    

    def handle_human_detect(self):
        self.get_logger().info('human_detect.')

        self.art_index = self.goal_poses_art.index(self.current_art) #현재 작품이 몇번째인지

        self.descript_point = self.point[self.art_index] # i를 받은 작품설명 위치

        goal_pose , orientation = self.descript_point # 작품설명위치의 위치와 자세

        goal_x, goal_y, goal_z = goal_pose
        goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w = orientation

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_x
        goal_pose_msg.pose.position.y = goal_y
        goal_pose_msg.pose.position.z = goal_z
        goal_pose_msg.pose.orientation.x = goal_orientation_x
        goal_pose_msg.pose.orientation.y = goal_orientation_y
        goal_pose_msg.pose.orientation.z = goal_orientation_z
        goal_pose_msg.pose.orientation.w = goal_orientation_w

        self.get_logger().info('Navigating to goal: ({}, {}, {})'.format(goal_x, goal_y, goal_z))

        # 목표 위치로 이동
        self.navigator.goToPose(goal_pose_msg)

        start_time = time.time()
        timeout = 12  # 12초 타임아웃

        while not self.navigator.isTaskComplete():
            if time.time() - start_time > timeout:
                self.navigator.cancelTask()
                self.get_logger().warn('Navigation to goal timed out!')
                break
            time.sleep(0.1)

        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived at the  location!')

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation to location was canceled!')

        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation to location failed!')
        

    def handle_guide(self, description):
        self.get_logger().info('Robot guiding to description location.')

        self.art_index = self.goal_poses_art.index(description) #작품명이 몇번째인지 확인

        self.descript_point = self.point[self.art_index] # i를 받은 작품설명 위치

        goal_pose , orientation = self.descript_point # 작품설명위치의 위치와 자세

        goal_x, goal_y, goal_z = goal_pose
        goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w = orientation

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_x
        goal_pose_msg.pose.position.y = goal_y
        goal_pose_msg.pose.position.z = goal_z
        goal_pose_msg.pose.orientation.x = goal_orientation_x
        goal_pose_msg.pose.orientation.y = goal_orientation_y
        goal_pose_msg.pose.orientation.z = goal_orientation_z
        goal_pose_msg.pose.orientation.w = goal_orientation_w

        self.navigator.goToPose(goal_pose_msg)

        start_time = time.time()
        timeout = 90  # 90초 타임아웃

        while not self.navigator.isTaskComplete():
            if time.time() - start_time > timeout:
                self.navigator.cancelTask()
                self.get_logger().warn('Navigation to goal timed out!')
                break
            time.sleep(0.1)

        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived at the location!')

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation to location was canceled!')

        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation to location failed!')

        self.current_state = 'guide at {}'.format(description)

        self.publish_robot_state()


    def comeback_to_patrol(self):

        # 가까운 웨이포인트로 이동
        goal_pose, orientation = self.goal_poses[self.art_index]

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

        self.navigator.goToPose(goal_pose_msg)

        start_time = time.time()
        timeout = 12  # 12초 타임아웃

        while not self.navigator.isTaskComplete():
            if time.time() - start_time > timeout:
                self.navigator.cancelTask()
                self.get_logger().warn('Navigation to goal timed out!')
                break
            time.sleep(0.1)

        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived at the description location!')

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation to description location was canceled!')

        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation to description location failed!')

        
        # 다음 웨이포인트 설정
        # 원래 방향이 위로가는거면  +1을 해서 위로 가고, 만약 6번 웨이포인트면 -1을 하고 아래로
        # 원래 방향이 아래가는거면  -11을 해서 아래로 가고, 만약 1번 웨이포인트면 +1을 하고 위로
        
        if self.forward_patrol == True:
            if self.art_index is not [5]:
                self.index = self.art_index + 1
            else:
                self.index = self.art_index - 1
                self.forward_patrol == False
        else:
            if self.art_index is not [0]:
                self.index = self.art_index - 1
            else:
                self.index = self.art_index + 1
                self.forward_patrol == True


def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
