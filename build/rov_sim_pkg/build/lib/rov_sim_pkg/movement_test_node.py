import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import time
import math

class MovementTestNode(Node):
    def __init__(self):
        super().__init__('movement_test_node')
        self.publisher_ = self.create_publisher(Twist, '/esp32/gyro_accel_data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('MovementTestNode has been started.')
        self.timer_period = 1 / 30  # 30 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.sequence = [
            ('ccw_yaw', 2.0), ('cw_yaw', 2.0),
            ('pitch_down', 2.0), ('pitch_up', 2.0),
            ('forward', 2.0), ('backward', 2.0),
            ('lateral_left', 2.0), ('lateral_right', 2.0),
            ('up', 2.0), ('down', 2.0)
        ]
        self.current_step = 0
        self.start_time = time.time()
        self.reset_transform()

    def timer_callback(self):
        if self.current_step < len(self.sequence):
            movement, duration = self.sequence[self.current_step]
            elapsed_time = time.time() - self.start_time

            msg = Twist()

            if movement == 'ccw_yaw':
                msg.angular.z = -1.0  # Simulate CCW yaw
            elif movement == 'cw_yaw':
                msg.angular.z = 1.0  # Simulate CW yaw
            elif movement == 'pitch_down':
                msg.angular.y = -1.0  # Simulate pitch down
            elif movement == 'pitch_up':
                msg.angular.y = 1.0  # Simulate pitch up
            elif movement == 'forward':
                msg.linear.x = 1.0  # Simulate forward
            elif movement == 'backward':
                msg.linear.x = -1.0  # Simulate backward
            elif movement == 'lateral_left':
                msg.linear.y = -1.0  # Simulate lateral left
            elif movement == 'lateral_right':
                msg.linear.y = 1.0  # Simulate lateral right
            elif movement == 'up':
                msg.linear.z = 1.0  # Simulate up
            elif movement == 'down':
                msg.linear.z = -1.0  # Simulate down

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing {movement} movement: {msg}')

            if elapsed_time >= duration:
                self.start_time = time.time()
                self.current_step += 1
                self.reset_position()

        else:
            self.current_step = 0
            self.start_time = time.time()

    def reset_position(self):
        self.get_logger().info('Resetting position...')
        self.reset_transform()

    def reset_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = self.euler_to_quaternion(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published transform reset to origin.')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    movement_test_node = MovementTestNode()
    rclpy.spin(movement_test_node)
    movement_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
