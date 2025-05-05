import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 

class TutorialTopic_1_2_Mock(Node):
    def __init__(self):
        super().__init__('mock_node_q1_2')
        self.pub1 = self.create_publisher(
            String,
            '/tutorial/basic_topic',
            10
        )
        self.create_timer(
            0.2,
            self.timer_callback
        )

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World!"
        self.pub1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    onboarding_topic_1_2_mock = TutorialTopic_1_2_Mock()
    rclpy.spin(onboarding_topic_1_2_mock)
    rclpy.shutdown()
