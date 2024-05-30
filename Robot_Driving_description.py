import rclpy as rp
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ros_package_msgs.srv import CommandString
from ros_package_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
import threading
import time , asyncio
import routes


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
        self.current_point = 0

        # 순찰 방향을 위한 플래그
        self.forward_patrol = True

        # 주기적으로 로봇 상태를 퍼블리시
        self.state_check_timer = self.create_timer(1.0, self.publish_robot_state)

        # 순찰 경로 정의
        self.route_forward = routes.route_forward

        self.route_reverse = routes.route_reverse
        
        self.route_forward_segments = routes.route_forward_segments
        
        self.route_reverse_segments = routes.route_reverse_segments
        
        self.route_forward_description = routes.route_forward_description

        self.route_resverse_description = routes.route_forward_description
        
        
        self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        self.patrol_work()
    
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

    def go_to_1(self):
        self.follow_route_segment(0)

    def follow_route_segment(self, segment_index, max_retries=3):
        current_route = self.route_forward_segments[segment_index]
        for pt in current_route:
            goal_pose = self.set_goal_pose(*pt)
            self.navigator.goToPose(goal_pose)
            retries = 0

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info('Distance remaining: {:.2f}'.format(feedback.distance_remaining))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Reached goal: ({}, {})'.format(pt[0], pt[1]))
                self.current_point = segment_index
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