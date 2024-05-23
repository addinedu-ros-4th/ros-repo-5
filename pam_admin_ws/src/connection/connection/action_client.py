import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from msg_pkg.action import Signal

class SignalActionClient(Node):
    def __init__(self):
        super().__init__('signal_action_client')
        self._action_client = ActionClient(self, Signal, 'signal')

    def send_goal(self, word):
        goal_msg = Signal.Goal()
        goal_msg.input_word = word

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.output_word}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)
    action_client = SignalActionClient()
    word = 'Hello, ROS 2'
    action_client.send_goal(word)
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()