import rclpy as rp
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
import threading
import time , asyncio
import ros_package.routes as routes

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
        self.current_segment_index = 0
        self.current_point = 0

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 주기적으로 로봇 상태를 퍼블리시
        self.state_check_timer = self.create_timer(1.0, self.publish_robot_state)

        # 순찰 경로 정의
              
        self.route_forward_segments = routes.route_forward_segments
        
        self.route_reverse_segments = routes.route_reverse_segments
        
        self.route_forward_description = routes.route_forward_description

        self.route_reverse_description = routes.route_forward_description
        
        self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        # 스타트 신호
        self.publish_robot_state()

        # 1번 웨이포인트로 이동
        self.patrol_work()

        # 멈춤상태 신호
        self.publish_robot_state()
        
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

    def follow_description_route(self, description_segments, segment_index, max_retries=3):
        current_route = description_segments[segment_index]
        for pt in current_route:
            if isinstance(pt, (list, tuple)) and len(pt) == 7:                
                goal_pose = self.set_goal_pose(*pt)
            else:
                continue

            self.navigator.goToPose(goal_pose)
            retries = 0

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info('Distance remaining in description route: {:.2f}'.format(feedback.distance_remaining))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Reached description goal: ({}, {})'.format(pt[0], pt[1]))
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Description goal was canceled, exiting.')
                return
            elif result == TaskResult.FAILED:
                retries += 1
                if retries >= max_retries:
                    self.get_logger().info('Failed to reach description goal: ({}, {}), max retries reached, exiting.'.format(pt[0], pt[1]))
                    break
                self.get_logger().info('Failed to reach description goal: ({}, {}), retrying...'.format(pt[0], pt[1]))


    def follow_route_segment(self, current_route_segments, segment_index, max_retries=3):
        current_route = current_route_segments[segment_index]
        for pt in current_route:
            if isinstance(pt, (list, tuple)) and len(pt) == 7:                
                goal_pose = self.set_goal_pose(*pt)
                self.navigator.goToPose(goal_pose)
                retries = 0
            
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info('Distance remaining: {:.2f}'.format(feedback.distance_remaining))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if current_route_segments == self.route_forward_segments:
                    self.current_point = segment_index + 1
                elif current_route_segments == self.route_reverse_segments:
                    self.current_point = 5 - segment_index
                self.get_logger().info('Reached goal: ({}, {}), current_point: {}'.format(pt[0], pt[1], self.current_point))
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled, exiting.')
                return
            elif result == TaskResult.FAILED:
                retries += 1
                if retries >= max_retries:
                    self.get_logger().info('Failed to reach goal: ({}, {}), max retries reached, exiting.'.format(pt[0], pt[1]))
                    break
                self.get_logger().info('Failed to reach goal: ({}, {}), retrying...'.format(pt[0], pt[1]))


    def patrol_work(self):
        self.current_state = 'Patrolling'
        self.publish_robot_state()

        # 현재 경로와 방향 선택
        current_route_segments = self.route_forward_segments if self.forward_patrol else self.route_reverse_segments
        
        # 경로의 현재 세그먼트 따라가기
        self.follow_route_segment(current_route_segments, self.current_segment_index)

        # 세그먼트 인덱스 업데이트
        self.current_segment_index += 1

        # 경로의 끝에 도달하면 순찰 방향을 반대로 전환하고 인덱스 초기화
        if self.current_segment_index >= len(current_route_segments):
            self.forward_patrol = not self.forward_patrol
            self.current_segment_index = 0

        # 상태를 업데이트
        self.current_state = 'arrive at segment {}'.format(self.current_segment_index)
                
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
        time.sleep(10)
        print('ok')
        
    def handle_description(self):
        self.get_logger().info('Robot stopped for description.')

    def handle_guide(self, description):
        self.get_logger().info('Robot guiding.')

    def comeback_to_patrol(self):
        self.patrolling = True
        self.current_state = 'Patrolling'
        # self.patrol_work()


def main(args=None):
    rp.init(args=args)

    robot_driving = RobotDriver()

    rp.spin(robot_driving)

    robot_driving.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()