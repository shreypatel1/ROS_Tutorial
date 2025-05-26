import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TutorialTopic_4_2_Helper(Node):
    def __init__(self):
        super().__init__('tutorial_4_2_helper')
        self.stbd_pub = self.create_publisher(
            Float64,
            '/stinger/thruster_stbd/cmd_thrust',
            10
        )

        self.port_pub = self.create_publisher(
            Float64,
            '/stinger/thruster_port/cmd_thrust',
            10
        )
    
        self.create_timer(
            0.5,
            self.forward_callback
        )
    
    def forward_callback(self):
        msg = Float64()
        msg.data = 5.0
        self.stbd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TutorialTopic_4_2_Helper()
    rclpy.spin(node)
