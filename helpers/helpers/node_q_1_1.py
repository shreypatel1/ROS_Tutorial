import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 

class TutorialTopic_1_1_Mock(Node):
    def __init__(self):
        super().__init__('mock_node_q1_1')
        self.pub1 = self.create_publisher(
            String,
            '/tutorial/StringPub',
            10
        )
        self.pub2 = self.create_publisher(
            Odometry,
            '/tutorial/OdometryPub',
            10
        )
        self.pub3 = self.create_publisher(
            Twist,
            '/tutorial/MysteryPub',
            10
        )

        self.string_timer = self.create_timer(
            0.5,
            self.timer_callback
        )

    def timer_callback(self):
        msg = String()
        msg.data = "mrg"
        self.pub1.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    onboarding_topic_1_1_mock = TutorialTopic_1_1_Mock()
    rclpy.spin(onboarding_topic_1_1_mock)
    rclpy.shutdown()
