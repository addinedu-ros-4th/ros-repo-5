import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class SecurityPatrol(Node):
    def __init__(self):
        super().__init__('security_patrol')
        self.navigator = BasicNavigator()

        # Define the security route with positions and orientations for forward and reverse routes
        self.route_forward = [
            (0.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(0))),
            (6.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (0.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (0.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(180))),
            (0.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
        ]

        self.route_reverse = [
            (6.0, -18.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (0.0, -12.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (0.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(90))),
            (6.0, -6.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
            (6.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(180))),
            (0.0, 0.0, 0.0, *self.euler_to_quaternion(math.radians(-90))),
        ]

        # Set the initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0  # Set to the starting point of the new route
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

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

    def patrol_route(self):
        routes = [self.route_forward, self.route_reverse]
        current_route_index = 0

        while rclpy.ok():
            # Get the current route
            current_route = routes[current_route_index]

            # Navigate to each goal in the route
            for pt in current_route:
                goal_pose = self.set_goal_pose(*pt)
                self.navigator.goToPose(goal_pose)

                # Monitor the progress and provide feedback
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        self.get_logger().info('Distance remaining: {:.2f}'.format(feedback.distance_remaining))

                    # # Check for stuck condition
                    # if self.navigator.getElapsedTime() > Duration(seconds=180.0):
                    #     self.get_logger().info('Navigation has exceeded timeout of 180s, canceling request.')
                    #     self.navigator.cancelTask()
                    #     break

                # Handle task result
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Reached goal: ({}, {})'.format(pt[0], pt[1]))
                elif result == TaskResult.CANCELED:
                    self.get_logger().info('Goal was canceled, exiting.')
                    return
                elif result == TaskResult.FAILED:
                    self.get_logger().info('Failed to reach goal: ({}, {}), retrying...'.format(pt[0], pt[1]))
                    break

            # Toggle the route index
            current_route_index = (current_route_index + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrol()
    node.patrol_route()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
