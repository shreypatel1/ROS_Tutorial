import rclpy
from rclpy.node import Node
from autograder.utils import grader
import subprocess

class Question_4_4_Grader(Node):
    def __init__(self):
        super().__init__('question_4_4_grader')
        self.declare_parameter('imu0_config', [False])
        self.declare_parameter('odom0_config', [False])
        ekf_imu_params = self.get_parameter('imu0_config').get_parameter_value().bool_array_value
        ekf_gps_params = self.get_parameter('odom0_config').get_parameter_value().bool_array_value
        self.verify_4_4_a(ekf_imu_params)
        self.verify_4_4_b()
        self.verify_4_4_c(ekf_gps_params)

    def verify_4_4_a(self, ekf_params):
        grader.verify_answer(15, len(ekf_params), '4.4.a Correct IMU config array length', self)
        result = 15 == len(ekf_params)
        correct = False
        if result:
            answer_array = [False, False, False, True, True, True, False, False, False, True, True, True, False, False, False]
            for answer, student in zip(ekf_params, answer_array):
                if answer != student:
                    break
            else:
                correct = True
        grader.verify_answer(True, correct, '4.4.a Correct IMU config params', self)

    def verify_4_4_c(self, ekf_params):
        grader.verify_answer(15, len(ekf_params), '4.4.c Correct GPS config array length', self)
        result = 15 == len(ekf_params)
        correct = False
        if result:
            answer_array = [True, True, False, False, False, False, False, False, False, False, False, False, False, False, False]
            for answer, student in zip(ekf_params, answer_array):
                if answer != student:
                    break
            else:
                correct = True
        grader.verify_answer(True, correct, '4.4.c Correct GPS config params', self)
    
    def verify_4_4_b(self):
        process = subprocess.run(['ros2', 'topic', 'info', '-v', '/stinger/imu/data'],
                                   capture_output=True,
                                   text=True)
        grader.verify_answer(lambda x: 'navsat_transform_node' in x, process.stdout, '4.4.b Correct IMU Remap', self)
        process = subprocess.run(['ros2', 'topic', 'info', '-v', '/stinger/gps/fix'],
                                   capture_output=True,
                                   text=True)
        grader.verify_answer(lambda x: 'navsat_transform_node' in x, process.stdout, '4.4.b Correct GPS Remap', self)

def main(args=None):
    rclpy.init(args=args)
    node = Question_4_4_Grader()
    rclpy.spin_once(node)
    rclpy.shutdown()