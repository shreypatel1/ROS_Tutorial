import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from autograder.utils import grader
import numpy as np

MAX_INTERATIONS = 1000

class Question_4_3_Grader(Node):
    def __init__(self):
        super().__init__('qustion_4_3_grader')

        ground_truth = Subscriber(
            self,
            Odometry,
            '/ground_truth/odometry',
        )

        student_odometry = Subscriber(
            self,
            Odometry,
            '/stinger/odometry',
        )

        self.ts = ApproximateTimeSynchronizer(
            [ground_truth, student_odometry],
            queue_size=10,
            slop=0.1,
        )
        self.current_iteration = 0

        self.ts.registerCallback(self.grade)
        # 0 - pose x 
        # 1 - pose y 
        # 2 - pose z 
        # 3 - twist x 
        # 4 - twist y 
        # 5 - twist z 
        # 6 - orientation roll
        # 7 - orientation pitch
        # 8 - orientation yaw
        # 9 - header frame id
        # 10 - child frame id
        # 11 - angular velocity x
        # 12 - angular velocity y
        # 13 - angular velocity z
        self.good = np.ones(14) * MAX_INTERATIONS 
    
    def grade(self, ground_truth: Odometry, student_odometry: Odometry):
        if self.current_iteration > MAX_INTERATIONS:
            self.correct()
            return
        self.get_logger().info(f"Iteration: {self.current_iteration}")
        self.current_iteration += 1
        ground_truth_pose = ground_truth.pose.pose
        student_pose = student_odometry.pose.pose
        ground_truth_twist = ground_truth.twist.twist
        student_twist = student_odometry.twist.twist
        if not grader.within_margin(ground_truth_pose.position.x, student_pose.position.x, 0.3):
            self.good[0] -= 1
        if not grader.within_margin(ground_truth_pose.position.y, student_pose.position.y, 0.3):
            self.good[1] -= 1
        if not grader.within_margin(0.0, student_pose.position.z, 0.3):
            self.good[2] -= 1
        if not grader.within_margin(ground_truth_twist.linear.x, student_twist.linear.x, 0.3):
            self.good[3] -= 1
        if not grader.within_margin(ground_truth_twist.linear.y, student_twist.linear.y, 0.3):
            self.good[4] -= 1
        if not grader.within_margin(0.0, student_twist.linear.z, 0.3):
            self.good[5] -= 1
        angle_diff = grader.euler_angle_diff(ground_truth_pose.orientation, student_pose.orientation)
        if angle_diff[0] > 0.3:
            self.good[6] -= 1
        if angle_diff[1] > 0.3:
            self.good[7] -= 1
        if angle_diff[2] > 0.3:
            self.good[8] -= 1
        if not student_odometry.header.frame_id == 'odom':
            self.good[9] -= 1
        if not student_odometry.child_frame_id == 'base_link':
            self.good[10] -= 1
        if not grader.within_margin(ground_truth_twist.angular.x, student_twist.angular.x, 0.3):
            self.good[11] -= 1
        if not grader.within_margin(ground_truth_twist.angular.y, student_twist.angular.y, 0.3):
            self.good[12] -= 1
        if not grader.within_margin(ground_truth_twist.angular.z, student_twist.angular.z, 0.3):
            self.good[13] -= 1
        
    def correct(self):
        self.good = (self.good / MAX_INTERATIONS) > 0.5
        grader.verify_answer(True, self.good[0], "4.3.b Correct pose x", self)
        grader.verify_answer(True, self.good[1], "4.3.b Correct pose y", self)
        grader.verify_answer(True, self.good[2], "4.3.b Correct pose z", self)
        grader.verify_answer(True, self.good[3], "4.3.b Correct twist x", self)
        grader.verify_answer(True, self.good[4], "4.3.b Correct twist y", self)
        grader.verify_answer(True, self.good[5], "4.3.b Correct twist z", self)
        grader.verify_answer(True, self.good[6], "4.3.b Correct orientation roll", self)
        grader.verify_answer(True, self.good[7], "4.3.b Correct orientation pitch", self)
        grader.verify_answer(True, self.good[8], "4.3.b Correct orientation yaw", self)
        grader.verify_answer(True, self.good[9], "4.3.b Correct header frame id", self)
        grader.verify_answer(True, self.good[10], "4.3.b Correct child frame id", self)
        grader.verify_answer(True, self.good[11], "4.3.b Correct angular velocity x", self)
        grader.verify_answer(True, self.good[12], "4.3.b Correct angular velocity y", self)
        grader.verify_answer(True, self.good[13], "4.3.b Correct angular velocity z", self)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Question_4_3_Grader()
    rclpy.spin(node)
