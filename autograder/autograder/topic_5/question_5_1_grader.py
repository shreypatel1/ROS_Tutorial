import rclpy
from rclpy.node import Node
from stinger_controller.velocity_controller import VelocityController
from stinger_controller.throttle_controller import  ThrottleController
from rclpy.time import Duration
from autograder.utils import grader
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.executors import MultiThreadedExecutor


def verify_q51a():
    node = VelocityController()
    wrench_pub: Publisher = node.wrench_pub
    grader.verify_answer('WrenchStamped', wrench_pub.msg_type.__qualname__, 'Q5.1.a Check Message Type Wrench Pub')
    grader.verify_answer('/cmd_wrench', wrench_pub.topic_name, 'Q5.1.a Check Topic Name Wrench Pub')
    cmd_vel_sub: Subscription = node.cmd_vel_sub
    grader.verify_answer('Twist', cmd_vel_sub.msg_type.__qualname__, 'Q5.1.a Check Message Type Cmd Vel Sub')
    grader.verify_answer('/cmd_vel', cmd_vel_sub.topic_name, 'Q5.1.a Check Topic Name Cmd Vel Sub')
    grader.verify_answer('cmd_vel_callback', cmd_vel_sub.callback.__name__, 'Q5.1.a Check Topic Callback Cmd Vel Sub')
    odom_sub: Subscription = node.odom_sub
    grader.verify_answer('Odometry', odom_sub.msg_type.__qualname__, 'Q5.1.a Check Message Type Odom Sub')
    grader.verify_answer('/odometry/filtered', odom_sub.topic_name, 'Q5.1.a Check Topic Name Odom Sub')
    grader.verify_answer('odometry_callback', odom_sub.callback.__name__, 'Q5.1.a Check Topic Callback Odom Sub')
    node.destroy_node()

def verify_q51bcdef():
    node = VelocityController()
    node.prev_time = node.get_clock().now()
    mock_twist = Twist()
    mock_twist.linear.x = 1.0
    mock_twist.angular.z = 0.5
    node.prev_error_surge = 0.4
    node.Kp_surge = 3
    node.Ki_surge = 4
    node.Kd_surge = 5 
    node.Kp_yaw = 6
    node.Ki_yaw = 7
    node.Kd_yaw = 8
    node.cmd_vel_callback(mock_twist)
    node.get_clock().now = lambda: node.prev_time + Duration(seconds=0.1)
    fake_odometry = Odometry()
    fake_odometry.twist.twist.linear.x = 0.5
    fake_odometry.twist.twist.angular.z = 0.2
    error, P_surge, I_surge, D_surge, control_yaw, output_force = node.odometry_callback(fake_odometry)
    grader.verify_answer(lambda x: np.isclose(x, 0.5, atol=0.001), error, 'Q5.1.b Check Surge Error Calculation')
    grader.verify_answer(lambda x: np.isclose(x, 1.5, atol=0.001), P_surge, 'Q5.1.c Check Surge Proportional Calculation')
    grader.verify_answer(lambda x: np.isclose(x, 0.2, atol=0.001), I_surge, 'Q5.1.d Check Surge Integral Calculation')
    grader.verify_answer(lambda x: np.isclose(x, 5, atol=0.001), D_surge, 'Q5.1.e Check Surge Derivative Calculation')
    fake_odometry.twist.twist.linear.x = 0.1
    error, P_surge, I_surge, D_surge, control_yaw, output_force = node.odometry_callback(fake_odometry)
    grader.verify_answer(lambda x: np.isclose(x, 20, atol=0.001), D_surge, 'Q5.1.e Check Surge Derivative Calculation Multi-iteration')
    grader.verify_answer(lambda x: np.isclose(x, 2.22, atol=0.001), control_yaw, 'Q5.1.f Check Control Yaw Calculation')
    grader.verify_answer(lambda x: np.isclose(x, 23.26, atol=0.01), output_force.wrench.force.x, 'Q5.1.f Check Output Force Message')
    grader.verify_answer(lambda x: np.isclose(x, 2.22, atol=0.01), output_force.wrench.torque.z, 'Q5.1.f Check Output Torque Message')
    node.destroy_node()

class TestControllerTuning(Node):
    def __init__(self):
        super().__init__('test_controller_tuning')
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.rmse_total_surge = 0
        self.rmse_total_yaw = 0
        self.current_observation_number = 0
        self.total_observations = 400
        self.create_timer(0.2, self.timer_callback)
    
    def odometry_callback(self, msg: Odometry):
        if self.current_observation_number >= self.total_observations:
            self.total_observations -= 50
            rmse_surge = np.sqrt(self.rmse_total_surge / self.total_observations)
            rmse_yaw = np.sqrt(self.rmse_total_yaw / self.total_observations)
            self.get_logger().info(f"RMSE_surge: {rmse_surge}, RMSE_yaw:  {rmse_yaw}. Need to be  < 0.075 to pass tests.")
            grader.verify_answer(lambda x: x < 0.075, rmse_surge, 'Q5.1.g Check Controller Tuning Surge')
            grader.verify_answer(lambda x: x < 0.075, rmse_yaw, 'Q5.1.g Check Controller Tuning Yaw')
            rclpy.shutdown()
            return
        self.get_logger().info(f"Iteration {self.current_observation_number}/{self.total_observations}")
        self.current_observation_number += 1
        if self.current_observation_number <  50:
            return
        self.rmse_total_surge += (0.3 -  msg.twist.twist.linear.x) **  2 
        self.rmse_total_yaw += (0.2 -  msg.twist.twist.angular.z) **  2 
    
    def timer_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3
        cmd_vel.angular.z = 0.2
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    verify_q51a()
    verify_q51bcdef()
    node = TestControllerTuning()
    controller_node = VelocityController()
    throttle_controller_node = ThrottleController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(controller_node)
    executor.add_node(throttle_controller_node)
    executor.spin()

