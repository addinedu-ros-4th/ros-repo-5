import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from msg_pkg.action import Signal

class SignalActionServer(Node):
    def __init__(self):
        super().__init__('signal_action_server')
        self._action_server = ActionServer(
            self,
            Signal,
            'signal',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request: {goal_handle.request.input_word}')
        feedback_msg = Signal.Feedback()
        feedback_msg.feedback = 'Processing request'
        goal_handle.publish_feedback(feedback_msg)
        
        result = Signal.Result()
        result.output_word = goal_handle.request.input_word
        
        goal_handle.succeed()
        
        return result

def main(args=None):
    rclpy.init(args=args)
    server = SignalActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()