from stinger_perception.detection import Detection
from autograder.utils import grader
import rclpy
import numpy as np
from ament_index_python.packages import get_package_share_directory
import cv2
import os


def test61ab():
    node = Detection()
    grader.verify_answer(True, node.question_1, 'Q6.1.a Check Value understanding')
    grader.verify_answer(False, node.question_2, 'Q6.1.a Check Saturation understanding')
    grader.verify_answer(lambda x: 70 <= x and x <= 110, node.question_3, 'Q6.1.a Check Hue understanding')
    
    acceptable_red_lower_bound = np.array([0, 0, 0])
    acceptable_red_upper_bound = np.array([30, 255, 255])
    acceptable_green_lower_bound = np.array([40, 0, 0])
    acceptable_green_upper_bound = np.array([100, 255, 255])
    
    node.hsv = np.ones((480, 360, 3))
    node.gate_detection_cv()


    grader.verify_answer(lambda x: np.all(acceptable_red_lower_bound <= x), node.red_lower, 'Q6.1.b Test red lower bound')
    grader.verify_answer(lambda x: np.all(acceptable_red_upper_bound >= x), node.red_lower, 'Q6.1.b Test red upper bound')
    grader.verify_answer(lambda x: np.all(acceptable_green_lower_bound <= x), node.green_lower, 'Q6.1.b Test green lower bound')
    grader.verify_answer(lambda x: np.all(acceptable_green_upper_bound >= x), node.green_upper, 'Q6.1.b Test green upper bound')

    node.destroy_node()

def test61c():
    node = Detection()
    # Get absolute path to the package
    pkg_share = get_package_share_directory('autograder')
    # [754, 338, 45]
    # Path to image
    img_path = os.path.join(pkg_share, 'resources', 'mask.png')
    image = cv2.imread(img_path, cv2.IMREAD_COLOR)
    mask = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    node.frame = np.ones((480, 360, 3))
    result = node.find_circles(mask)
    one_result = (len(result) == 1)
    grader.verify_answer(True, one_result, 'Q6.1.d Verify One Detection')
    acceptable_lower_bound = np.array([720, 300, 30])
    acceptable_upper_bound = np.array([780, 370, 60])
    if not one_result:
        grader.verify_answer(True, False, 'Q6.1.d. Correct detection')
    else:
        grader.verify_answer(lambda x: np.all(acceptable_lower_bound <= x) and np.all(acceptable_upper_bound >= x), result[0], 'Q6.1.d. Correct detection')


def main(args=None):
    rclpy.init()
    test61ab()
    test61c()
