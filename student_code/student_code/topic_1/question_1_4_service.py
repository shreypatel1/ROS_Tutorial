# import rclpy
# from rclpy.node import Node
# import pandas as pd

# class TutorialTopic_1_4_Service(Node):
#     def __init__(self):
#         super().__init__('question_1_4_service')
#         tutorial_map = pd.DataFrame(
#             columns=['type', 'x_loc', 'y_loc']
#         )
#         tutorial_map.loc[len(tutorial_map)] = {
#             'type': 'red_buoy',
#             'x_loc': 3.0,
#             'y_loc': 4.0
#         }
#         tutorial_map.loc[len(tutorial_map)] = {
#             'type': 'green_buoy',
#             'x_loc': 0.0,
#             'y_loc': -1.0
#         }
#         tutorial_map.loc[len(tutorial_map)] = {
#             'type': 'black_buoy',
#             'x_loc': 2.0,
#             'y_loc': 10.0
#         }
#         tutorial_map.loc[len(tutorial_map)] = {
#             'type': 'yellow_buoy',
#             'x_loc': 1.0,
#             'y_loc': -8.0
#         }

#         self.gate_side_information_service = self.create_service(
#             GetBuoyLocation,
#             '/pontus/gate_side_information',
#             self.handle_gate_side_information
#         )