from autograder.utils import grader
from student_code.topic_3 import question_3_2

def main(args=None):
    answers = question_3_2.TutorialTopic_3_2()
    grader.verify_answer(2, answers.red_buoy_location.position.x, 'Q3.2.a Red x')
    grader.verify_answer(3, answers.red_buoy_location.position.y, 'Q3.2.a Red y')
    grader.verify_answer(-3, answers.yellow_buoy_location.position.x, 'Q3.2.a Yellow x')
    grader.verify_answer(1, answers.yellow_buoy_location.position.y, 'Q3.2.a Yellow y')

    grader.verify_answer(lambda x: abs(x - 0.9827937232473289) < 0.05, answers.red_buoy_angle, 'Q3.2.b Red angle')
    grader.verify_answer(lambda x: abs(x - 2.819842099193151) < 0.05, answers.yellow_buoy_angle, 'Q3.2.b Yellow angle')
