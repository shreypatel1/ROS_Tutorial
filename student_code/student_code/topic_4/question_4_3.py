import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
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

    def transfrom_imu(self, msg: Imu):
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

        msg_base_link = self.transfrom_imu(msg)
        if msg_base_link is None:
            self.get_logger().warn("Failed to get transform, skipping")
            return
        self.imu_debug_pub.publish(msg_base_link)

        # TODO: 4.3.a Odom Frame IMU
        ### STUDENT CODE HERE
        # Rotate the base_link measurements into the odom frame using the IMU's
        # own orientation, which is the rotation from base_link to odom.
        q = msg_base_link.orientation
        rot_matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

        linear_acc_odom = self.rotate_vector(rot_matrix, msg_base_link.linear_acceleration)
        ### END STUDENT CODE


        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        # TODO: 4.3.b IMU Dead Reckoning
        ### STUDENT CODE HERE
        # Integrate on the sensor stamp. This node runs on wall time but the sim
        # publishes /clock, so get_clock() would scale dt by the real time factor.
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return

        # In the odom frame gravity is purely vertical, so taking x and y drops it
        self.velocity += linear_acc_odom[:2] * dt
        self.position += self.velocity * dt

        # twist is expressed in child_frame_id, so rotate the velocity back to base_link
        vel_base_link = rot_matrix.T @ np.array([self.velocity[0], self.velocity[1], 0.0])

        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = msg_base_link.orientation
        odom_msg.twist.twist.linear.x = vel_base_link[0]
        odom_msg.twist.twist.linear.y = vel_base_link[1]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular = msg_base_link.angular_velocity

        self.odom_pub.publish(odom_msg)
        ### END STUDENT CODE

def main(args=None):
    rclpy.init(args=args)
    node = TutorialTopic_4_3()
    rclpy.spin(node)
