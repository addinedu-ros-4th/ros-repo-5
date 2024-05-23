import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.goals = {
            "1": (0.00426, -0.143, -0.00143, 0.0, 0.0, 0.0, 1.0),
            "2": (1.0, -0.143, -0.00137, 0.0, 0.0, 0.0, 1.0),
            "3": (1.0, -1.15, -0.00143, 0.0, 0.0, 0.0, 1.0),
            "4": (0.2, -1.15, -0.00143, 0.0, 0.0, 0.0, 1.0),
            "5": (0.2, -2.23, -0.00143, 0.0, 0.0, 0.0, 1.0),
            "6": (1.0, -2.23, -0.00143, 0.0, 0.0, 0.0, 1.0)
        }

        self.goal_subscription = self.create_service(SetBool, 'navigate_to_goal', self.goal_callback)

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

    def navigate_to_goal(self, goal_pose):
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining))

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

    def goal_callback(self, request, response):
        target_goal = str(request.data)
        if target_goal not in self.goals:
            self.get_logger().info('Invalid goal received')
            response.success = False
            return response

        current_goal = self.get_current_goal_number()
        path = self.get_path(current_goal, target_goal)

        for goal_number in path:
            goal_coords = self.goals[goal_number]
            goal_pose = self.set_goal_pose(*goal_coords)
            self.navigate_to_goal(goal_pose)

        response.success = True
        return response

    def get_current_goal_number(self):
        # Placeholder function to get the current goal number
        # This should be replaced with actual logic to determine the current goal
        return "1"

    def get_path(self, start, end):
        start_index = int(start)
        end_index = int(end)
        if start_index <= end_index:
            return [str(i) for i in range(start_index, end_index + 1)]
        else:
            return [str(i) for i in range(start_index, end_index - 1, -1)]

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
