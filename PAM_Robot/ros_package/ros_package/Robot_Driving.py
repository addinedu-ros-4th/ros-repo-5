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
        
        # # 기본적으로 navigator를 초기화
        # self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active()

        # 순찰 상태 관리
        self.current_state = 'Patrolling'
        self.patrolling = True
        self.command_received = False

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 목표 지점의 인덱스 초기화
        self.index = 0

        # 주기적으로 로봇 상태를 퍼블리시
        self.state_check_timer = self.create_timer(1.0, self.publish_robot_state)

        # 순찰 작업을 위한 스레드
        self.patrol_thread = threading.Thread(target=self.patrol_work)
        self.patrol_thread.start()

    
    def publish_robot_state(self):
        msg = RobotState()
        msg.command = self.current_state

        self.robot_state_pose_publisher.publish(msg)
        print(msg.command)
        # self.get_logger().info('Published robot state: {}'.format(msg.command))


    def patrol_work(self):
        self.goal_poses = [
            ((0.8, 0.8, 0.0), (0.0, 0.0, 0.0)),  # 첫 번째 목표 지점
            ((1.3, 4.0, 0.0), (0.0, 0.0, 0.0)),  # 두 번째 목표 지점
            ((6.0, 3.7, 0.0), (0.0, 0.0, 0.0)),  # 세 번째 목표 지점
            ((6.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 네 번째 목표 지점
            ((10.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 다섯 번째 목표 지점
            ((10.0, 3.3, 0.0), (0.0, 0.0, 0.0)),  # 여섯 번째 목표 지점
            ((15.0, 2.8, 0.0), (0.0, 0.0, 0.0)),  # 일곱 번째 목표 지점
            ((15.0, 0.0, 0.0), (0.0, 0.0, 0.0)),  # 여덟 번째 목표 지점
        ]

        self.goal_poses_art = ['피라미드','2','나폴레옹','스핑크스','오벨리스크','람세스','7','이시스'] # 작품위치에 따른 작품명
        self.current_art = None

        self.point = [
            ((0,0,0),(0,0,0,0)), # 첫번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 두번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 세번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 네번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 다섯번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 여섯번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 일곱번째 목표 지점의 작품설명위치
            ((0,0,0),(0,0,0,0)), # 여덟번째 목표 지점의 작품설명위치
        ]


        while rp.ok():
            if self.patrolling and not self.command_received:
                    if not self.patrolling:
                        break

                    self.current_state = 'Patrolling'
                    time.sleep(3)

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


                    # self.get_logger().info('Navigating to goal: ({}, {}, {})'.format(goal_x, goal_y, goal_z))

                    # # 목표 위치로 이동
                    # self.navigator.goToPose(goal_pose_msg)

                    # # 목표 지점에 도착하기를 기다림
                    # while not self.navigator.isTaskComplete() and not self.command_received:
                    #     time.sleep(0.1)

                    # if self.command_received:
                    #     self.navigator.cancelTask()
                    #     break

                    # result = self.navigator.getResult()
                    
                    # if result == TaskResult.SUCCEEDED:
                    #     self.get_logger().info('Goal succeeded at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))
                    self.current_state = 'arrive at {}'.format(self.goal_poses_art[self.index])
                    self.current_art = self.goal_poses_art[self.index]

                    # elif result == TaskResult.CANCELED:
                    #     self.get_logger().info('Goal was canceled at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

                    # elif result == TaskResult.FAILED:
                    #     self.get_logger().info('Goal failed at ({}, {}, {})!'.format(goal_x, goal_y, goal_z))

                    # 현재 목표 지점 인덱스 업데이트
                    if self.forward_patrol:
                        self.index += 1
                    else:
                        self.index -= 1

                    # 목표 지점 도착 후 3초 대기
                    time.sleep(3)

                    # 2번과 7번 웨이포인트에서는 커맨드를 무시하고 계속 진행
                    if self.index not in [1, 6] and self.command_received:
                        break


    def robot_command_callback(self, request, response):
        self.get_logger().info('Robot Command Server started')

        self.command_received = True
        
        if request.command == "human_detect":
            self.get_logger().info('Received human_detect')
            response.success = True
            response.message = 'human_detect'

            self.patrolling = False
            self.current_state = 'Human Detection'

            self.handle_human_detect()
 

        elif request.command == "description":
            self.get_logger().info('Received description')
            response.success = True
            response.message = 'description'

            self.patrolling = False
            self.current_state = 'Description'

            self.handle_description()
            

        elif request.command == "guide":
            self.get_logger().info('Received guide')
            response.success = True
            response.message = 'guide'

            self.patrolling = False
            self.current_state = 'Guiding'

            self.handle_guide(request.description)

            
        elif request.command == "comeback":
            self.get_logger().info('Received comeback')
            response.success = True
            response.message = 'comeback'

            self.patrolling = True
            self.current_state = 'Returning to Patrol'

            self.comeback_to_patrol()

        else:
            self.get_logger().error('Received Unknown')
            response.success = False
            response.message = 'Unknown'

        return response
    

    def handle_human_detect(self):
        self.get_logger().info('human_detect.')
        self.publish_robot_state()

        self.art_index = self.goal_poses_art.index(self.current_art) #현재 작품이 몇번째인지

        self.descript_point = self.point[self.art_index] # i를 받은 작품설명 위치

        # point 값을 이용해 목표 위치 설정
        # goal_x, goal_y, goal_z = point[0,1,2]
        # goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w = point[3,4,5,6]

        # goal_pose_msg = PoseStamped()
        # goal_pose_msg.header.frame_id = 'map'
        # goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        # goal_pose_msg.pose.position.x = goal_x
        # goal_pose_msg.pose.position.y = goal_y
        # goal_pose_msg.pose.position.z = goal_z
        # goal_pose_msg.pose.orientation.x = goal_orientation_x
        # goal_pose_msg.pose.orientation.y = goal_orientation_y
        # goal_pose_msg.pose.orientation.z = goal_orientation_z
        # goal_pose_msg.pose.orientation.w = goal_orientation_w

        # self.navigator.goToPose(goal_pose_msg)

        # while not self.navigator.isTaskComplete() and not self.command_received:
        #     time.sleep(0.1)

        # result = self.navigator.getResult()
        
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info('Arrived at the description location!')

        # elif result == TaskResult.CANCELED:
        #     self.get_logger().info('Navigation to description location was canceled!')

        # elif result == TaskResult.FAILED:
        #     self.get_logger().info('Navigation to description location failed!')
        

    def handle_description(self):
        self.get_logger().info('Robot stopped for description.')


    def handle_guide(self, description):
        self.get_logger().info('Robot guiding to description location.')
        self.publish_robot_state()

        self.art_index = self.goal_poses_art.index(description) #작품명이 몇번째인지 확인

        self.descript_point = self.point[self.art_index] # i를 받은 작품설명 위치

        # # point 값을 이용해 목표 위치 설정
        # goal_x, goal_y, goal_z = point[0,1,2]
        # goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w = point[3,4,5,6]

        # goal_pose_msg = PoseStamped()
        # goal_pose_msg.header.frame_id = 'map'
        # goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        # goal_pose_msg.pose.position.x = goal_x
        # goal_pose_msg.pose.position.y = goal_y
        # goal_pose_msg.pose.position.z = goal_z
        # goal_pose_msg.pose.orientation.x = goal_orientation_x
        # goal_pose_msg.pose.orientation.y = goal_orientation_y
        # goal_pose_msg.pose.orientation.z = goal_orientation_z
        # goal_pose_msg.pose.orientation.w = goal_orientation_w

        # self.navigator.goToPose(goal_pose_msg)

        # while not self.navigator.isTaskComplete() and not self.command_received:
        #     time.sleep(0.1)

        # result = self.navigator.getResult()
        
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info('Arrived at the description location!')

        # elif result == TaskResult.CANCELED:
        #     self.get_logger().info('Navigation to description location was canceled!')

        # elif result == TaskResult.FAILED:
        #     self.get_logger().info('Navigation to description location failed!')

        self.current_state = 'Human Detection'


    def comeback_to_patrol(self):

        self.index = self.art_index + 1

        # 가장 가까운 웨이포인트로 이동
        # goal_x, goal_y, goal_z = waypoint[0], waypoint[1], waypoint[2]
        # goal_pose_msg = PoseStamped()
        # goal_pose_msg.header.frame_id = 'map'
        # goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        # goal_pose_msg.pose.position.x = goal_x
        # goal_pose_msg.pose.position.y = goal_y
        # goal_pose_msg.pose.position.z = goal_z
        # goal_pose_msg.pose.orientation.w = 1.0

        # self.navigator.goToPose(goal_pose_msg)
        # while not self.navigator.isTaskComplete() and not self.command_received:
        #     time.sleep(0.1)

        # result = self.navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info('Arrived at the nearest waypoint!')

        # 순찰 작업을 위한 현재 웨이포인트 인덱스 설정

        if self.art_index in [0, 1, 2, 3]:
            self.forward_patrol = True
        else:
            self.forward_patrol = False

        self.patrolling = True
        self.command_received = False

        print("NOOOOO")



def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
