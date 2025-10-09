import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
from rclpy.time import Time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np
from tf2_ros import TransformListener, Buffer

np.set_printoptions(suppress=True)


class TutorialTopic_4_3(Node):
    def __init__(self):
        super().__init__('tutorial_topic_4_3')
        self.create_subscription(
            Imu,
            '/stinger/imu/data',
            self.imu_callback,
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/stinger/odometry',
            10
        )
        self.imu_debug_pub = self.create_publisher(
            Imu,
            '/debug',
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity = np.zeros(2)
        self.position = np.zeros(2)
        self.prev_time = None
        self.min_iterations_startup = 0
    
    def rotate_vector(self, rot_matrix, v):
            vec = np.array([v.x, v.y, v.z])
            return rot_matrix @ vec

    def transform_imu(self, msg: Imu):
        transform = None
        # Extract transform from imu to base link from tf tree
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except:
            return None

        # Extract transform rotation
        q = transform.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        rot_matrix = tf_transformations.quaternion_matrix(q_tf)[:3, :3]

        # Transform orientation into base_link frame
        imu_q = msg.orientation
        imu_q_np = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]
        transformed_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q_tf, imu_q_np),
            tf_transformations.quaternion_conjugate(q_tf)
        )

        # Transform angular velocity and linear acceleration into base_link frame

        ang_vel = self.rotate_vector(rot_matrix, msg.angular_velocity)
        lin_acc = self.rotate_vector(rot_matrix, msg.linear_acceleration)

        transformed_msg = Imu()
        transformed_msg.header.stamp = msg.header.stamp
        # Notice that the new frame is now in our base_link frame as desired
        transformed_msg.header.frame_id = 'base_link'
        transformed_msg.orientation.x = transformed_orientation[0]
        transformed_msg.orientation.y = transformed_orientation[1]
        transformed_msg.orientation.z = transformed_orientation[2]
        transformed_msg.orientation.w = transformed_orientation[3]
        transformed_msg.angular_velocity.x = ang_vel[0]
        transformed_msg.angular_velocity.y = ang_vel[1]
        transformed_msg.angular_velocity.z = ang_vel[2]
        transformed_msg.linear_acceleration.x = lin_acc[0]
        transformed_msg.linear_acceleration.y = lin_acc[1]
        transformed_msg.linear_acceleration.z = lin_acc[2]

        return transformed_msg

    def imu_callback(self, msg: Imu):
        # Ignore the first few publishes due to startup inconsistencies
        if self.min_iterations_startup < 10:
            self.min_iterations_startup += 1
            return

        msg_base_link = self.transform_imu(msg)
        if msg_base_link is None:
            self.get_logger().warn("Failed to get transform, skipping")
            return
        self.imu_debug_pub.publish(msg_base_link)

        # TODO: 4.3.a Odom Frame IMU
        ### STUDENT CODE HERE
        ### To correctly localize ourselves, we must convert our base_link measurements into the odom frame. With an IMU, this involves using the IMU's orientation to rotate the linear_acceleration and angular_velocity. Look at question_4_3.py, complete 4.3.a Odom Frame IMU by rotating the linear_acceleration and angular_velocity from msg_base_link with msg_base_link.orientation.
        # Hint: Use the code in transform_imu as reference.

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame=msg_base_link.header.frame_id,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except:
            return None

        q = transform.transform.rotation
        q_tf = [q.x, q.y, q.z, q.w]
        rot_matrix = tf_transformations.quaternion_matrix(q_tf)[:3, :3]

        base_q = msg_base_link.orientation
        base_q_np = [base_q.x, base_q.y, base_q.z, base_q.w]
        transformed_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q_tf, base_q_np),
            tf_transformations.quaternion_conjugate(q_tf)
        )

        ang_vel = self.rotate_vector(rot_matrix, msg_base_link.angular_velocity)
        lin_acc = self.rotate_vector(rot_matrix, msg_base_link.linear_acceleration)

        # # Subtract gravity in world frame
        # g = 9.80665
        # lin_acc[2] -= g

        # # Bias calibration on startup (average first 50 samples)
        # if not hasattr(self, "accel_bias"):
        #     if not hasattr(self, "accel_bias_accum"):
        #         self.accel_bias_accum = np.zeros(3)
        #         self.accel_bias_count = 0
        #     if self.accel_bias_count < 50:
        #         self.accel_bias_accum += lin_acc
        #         self.accel_bias_count += 1
        #         if self.accel_bias_count == 50:
        #             self.accel_bias = self.accel_bias_accum / self.accel_bias_count
        #             self.get_logger().info(f"Accel bias estimated: {self.accel_bias}")
        #         return
        # else:
        #     lin_acc -= self.accel_bias

        ### END STUDENT CODE


        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        # TODO: 4.3.b IMU Dead Reckoning
        ### STUDENT CODE HERE
        # Look at question_4_3.py, complete 4.3.b IMU Dead Reckoning. For this question, we want to publish to the topic /stinger/odometry an odometry message containing the following fields:
        # frame_id
        # child_frame_id
        # twist
        # pose
        # For all fields relating to heave, you can set it to 0. Also, disregard the covaraince fields. A topic called /ground_truth/odometry is given to you which you should use to verify your computation.
        # Hint: Use the variables defined in the constructor.

        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        current_time = Time.from_msg(msg.header.stamp) # self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        prev_velocity = self.velocity.copy()
        self.velocity += lin_acc[:2] * dt
        self.position += (prev_velocity * dt) + (0.5 * lin_acc[:2] * (dt**2))
        odom_msg.twist.twist.linear.x = self.velocity[0] #
        odom_msg.twist.twist.linear.y = self.velocity[1] #
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = ang_vel[0]
        odom_msg.twist.twist.angular.y = ang_vel[1]
        odom_msg.twist.twist.angular.z = ang_vel[2]
        odom_msg.pose.pose.position.x = self.position[0] #
        odom_msg.pose.pose.position.y = self.position[1] #
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = transformed_orientation[0]
        odom_msg.pose.pose.orientation.y = transformed_orientation[1]
        odom_msg.pose.pose.orientation.z = transformed_orientation[2]
        odom_msg.pose.pose.orientation.w = transformed_orientation[3]

        # self.get_logger().info(
        #     f"IMU frame: {msg.header.frame_id}, orientation: "
        #     f"[{msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f}], "
        #     f"linear_acc: [{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}]"
        # )


        self.odom_pub.publish(odom_msg)

        ### END STUDENT CODE

def main(args=None):
    rclpy.init(args=args)
    node = TutorialTopic_4_3()
    rclpy.spin(node)
