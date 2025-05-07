import rclpy
from autograder.utils import grader
from tutorial_msgs.srv import GetBuoyLocation
from student_code.topic_1.question_1_4_service import TutorialTopic_1_4_Service

def verify_service_message():
    # Verify Request field
    request = GetBuoyLocation.Request()
    grader.verify_answer(lambda x : 'buoy_name' in x, request.get_fields_and_field_types(), 'Q1.4.a Correct Request Parameter Name')
    grader.verify_answer('string', request.get_fields_and_field_types().get('buoy_name', None), 'Q1.4.a Correct Request Parameter Type')

    # Verify response field
    response = GetBuoyLocation.Response()
    grader.verify_answer(lambda x : 'x_pos' in x, response.get_fields_and_field_types(), 'Q1.4.a Correct Response Parameter Name x')
    grader.verify_answer(lambda x : 'y_pos' in x, response.get_fields_and_field_types(), 'Q1.4.a Correct Response Parameter Name y')
    grader.verify_answer('int32', response.get_fields_and_field_types().get('x_pos', None), 'Q1.4.a Correct Response Parameter Type x')
    grader.verify_answer('int32', response.get_fields_and_field_types().get('y_pos', None), 'Q1.4.a Correct Response Parameter Type y')
    grader.verify_answer(lambda x: 'found' in x, response.get_fields_and_field_types(), 'Q1.4.a Correct Response Parameter Name found')
    grader.verify_answer('boolean', response.get_fields_and_field_types().get('found', None), 'Q1.4.a Correct Response Parameter Type found')

def verify_service(service):
    # Verify service definition
    grader.verify_answer('GetBuoyLocation', service.srv_type.__qualname__, 'Q1.4.b Service Type')
    grader.verify_answer('/tutorial/get_buoy_location', service.srv_name, 'Q1.4.b Service Topic Name')

def verify_service_isloated(service_node):
    request = GetBuoyLocation.Request()
    response = GetBuoyLocation.Response()
    request.buoy_name = 'red_buoy'
    response = service_node.handle_get_buoy_location(request, response)
    grader.verify_answer((3.0, 4.0, True), (response.x_pos, response.y_pos, response.found), 'Q1.4.c Handle red_buoy request')
    request.buoy_name = 'green_buoy'
    response = service_node.handle_get_buoy_location(request, response)
    grader.verify_answer((0.0, -1.0, True), (response.x_pos, response.y_pos, response.found), 'Q1.4.c Handle green_buoy request')
    request.buoy_name = 'white_buoy'
    response = service_node.handle_get_buoy_location(request, response)
    grader.verify_answer(False, response.found, 'Q1.4.c Handle non existing request')
    # Inject hidden test case
    service_node.tutorial_map.loc[len(service_node.tutorial_map)] = {
        'type': 'test_buoy',
        'x_loc': 16.0,
        'y_loc': 18.0
    }
    request.buoy_name = 'test_buoy'
    response = service_node.handle_get_buoy_location(request, response)
    grader.verify_answer((16.0, 18.0, True), (response.x_pos, response.y_pos, response.found), 'Q1.4.c Handle hidden test case request')

def main(args=None):
    rclpy.init(args=args)
    verify_service_message()
    node = TutorialTopic_1_4_Service()
    verify_service(node.gate_side_information_service)
    verify_service_isloated(node)