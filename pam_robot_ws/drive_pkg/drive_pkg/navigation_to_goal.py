import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
                self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining))

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            
    def navigate_from_2_to_6(self):
        # 2번부터 8번까지 순차적으로 이동
        for goal_number in ["2", "3", "4", "5", "6"]:
            goal_coords = self.goals[goal_number]
            goal_pose = self.set_goal_pose(*goal_coords)
            self.navigate_to_goal(goal_pose)
    
    def navigate_from_5_to_1(self):
        # 7번부터 1번까지 역순으로 이동
        for goal_number in ["5", "4", "3", "2", "1"]:
            goal_coords = self.goals[goal_number]
            goal_pose = self.set_goal_pose(*goal_coords)
            self.navigate_to_goal(goal_pose)

            
def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()
    # node.navigate_from_5_to_1()
    node.navigate_from_2_to_6()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
