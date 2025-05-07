import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from student_code.topic_1 import question_1_3
from autograder.utils import grader
import time
from rclpy.executors import MultiThreadedExecutor

class Question1_3_Grader(Node):
    def __init__(self):
        super().__init__('question1_3_grader_node')
        self.counter_subscriber = self.create_subscription(
            Int32, 
            '/tutorial/counter25',
            self.counter_callback,
            10
        )
        self.expected = 0
        self.not_failed = True
        self.passed = False
        self.last_message_time = time.time()
        self.timeout_timer = self.create_timer(0.5, self.timeout_callback)
        self.get_logger().info("Test in progress...")

    def counter_callback(self, msg: Int32):
        self.last_message_time = time.time()
        if msg.data > 25 or msg.data < 0:
            self.not_failed = False
        elif msg.data != self.expected:
            self.not_failed = False
        elif msg.data == self.expected and msg.data == 25:
            self.passed = True
        else:
            self.expected += 1

    def timeout_callback(self):
        if time.time() - self.last_message_time > 1 or not self.not_failed:
            grader.verify_answer(True, self.not_failed and self.passed, 'Q1.3 Counter Node')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    student_node = question_1_3.TutorialTopic_1_3()
    executor = MultiThreadedExecutor()
    grader_node = Question1_3_Grader()
    executor.add_node(grader_node)
    executor.add_node(student_node)
    executor.spin()
