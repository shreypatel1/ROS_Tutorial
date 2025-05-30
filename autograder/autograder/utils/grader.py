from geometry_msgs.msg import Quaternion
import tf_transformations
import numpy as np

GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


def verify_answer(expected_condition, received_value, question_name, other=None):
    try:
        if callable(expected_condition):
            condition_met = expected_condition(received_value)
        else:
            condition_met = expected_condition == received_value

        if condition_met:
            if other is None:
                print(f"{GREEN}PASSED{RESET} {question_name}")
            else:
                other.get_logger().info(f"{GREEN}PASSED{RESET} {question_name}")
        else:
            if other is None:
                print(f"{RED}FAILED{RESET} {question_name}")
            else:
                other.get_logger().info(f"{RED}FAILED{RESET} {question_name}")
    except Exception as e:
        if other is None:
            print(f"{RED}ERROR{RESET} {question_name}")
            print(e)
        else:
            other.get_logger().info(f"{RED}ERROR{RESET} {question_name}")
            other.get_logger().info(f"{e}")

def within_margin(expected, actual, margin):
    if abs(expected) <= 0.1:
        return abs(actual) < margin
    return abs((expected - actual) / expected) < margin

def euler_angle_diff(angle_1: Quaternion, angle_2: Quaternion):
    q1 = [angle_1.x, angle_1.y, angle_1.z, angle_1.w]
    q2 = [angle_2.x, angle_2.y, angle_2.z, angle_2.w]

    R1 = tf_transformations.quaternion_matrix(q1)[:3, :3]
    R2 = tf_transformations.quaternion_matrix(q2)[:3, :3]

    R_diff = np.dot(R2, R1.T)

    euler_diff = tf_transformations.euler_from_matrix(R_diff, axes='sxyz')
    return np.abs(euler_diff)
