import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TutorialTopic_1_2(Node):
    def __init__(self):
        super().__init__('tutorial_node_q_1_2')

        ## TODO: Q1.2.a Creating a subscriber to a topic
        self.basic_topic_subscription = self.create_subscription(
            ### STUDENT CODE HERE

            ### END STUDENT CODE
        )
        
        # TODO: Q1.2.b Creating Publisher
        self.new_topic_publisher = self.create_publisher(
            ### STUDENT CODE HERE

            ### END STUDENT CODE
        )
        self.topic_string_message = None


    def topic_callback(self, msg: String):
        ## TODO: Q1.2.c Message Data
        self.topic_string_message = None
        ### STUDENT CODE HERE

        ### END STUDENT CODE

        # TODO: Q1.2.d Publishing Data  
        new_message = String()
        ### STUDENT CODE HERE

        ### END STUDENT CODE
    
def main(args=None):
    rclpy.init(args=args)
    node = TutorialTopic_1_2()
    rclpy.spin(node)
