from autograder.utils import grader
from tutorial_msgs.srv import GetBuoyLocation
import rosidl_parser.definition

def verify_service_message():
    # Verify Request field
    request = GetBuoyLocation.Request()
    grader.verify_answer(lambda x : 'buoy_name' in x, request.get_fields_and_field_types(), 'Q1.4.a Correct Request Parameter Name')
    grader.verify_answer('string', request.get_fields_and_field_types().get('buoy_name', None), 'Q1.4.a Correct Request Parameter Type')

    # Verify response field
    response = GetBuoyLocation.Response()
    grader.verify_answer(lambda x : 'x_pos' in x, response.get_fields_and_field_types(), 'Q1.4.a Correct Response Parameter Name x')
    grader.verify_answer(lambda x : 'y_pos' in x, response.get_fields_and_field_types(), 'Q1.4.a Correct Response Parameter Name y')
    grader.verify_answer('int32', response.get_fields_and_field_types().get('x_pos', None), 'Q1.4.a Correct Request Parameter Type x')
    grader.verify_answer('int32', response.get_fields_and_field_types().get('y_pos', None), 'Q1.4.a Correct Request Parameter Type y')

def main(args=None):
    verify_service_message()