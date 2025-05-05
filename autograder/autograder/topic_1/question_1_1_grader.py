from autograder.utils import grader
from student_code.topic_1 import question_1_1

def main(args=None):
    answers = question_1_1.TutorialTopic_1_1()
    grader.verify_answer(1, answers.num_nodes, 'Q1.1.a Nodes')
    grader.verify_answer("/mock_node_q1_1", answers.first_node_name, 'Q1.1.b Nodes Names')
    grader.verify_answer(5, answers.num_topics, 'Q1.1.c Topics')
    grader.verify_answer("geometry_msgs/msg/Twist", answers.topic_message_type, 'Q1.1.d Topic Info')
    grader.verify_answer("mrg", answers.string_message, 'Q1.1.e Topic Echo')
