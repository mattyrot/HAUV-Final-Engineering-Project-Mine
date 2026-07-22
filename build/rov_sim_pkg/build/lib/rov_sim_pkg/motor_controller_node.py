import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.subscription = self.create_subscription(
            Twist,
            '/esp32/bno055_data',
            self.bno055_callback,
            10)
        self.get_logger().info('MotorControllerNode has been started.')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_position = [0.0, 0.0, 0.0]  # x, y, z position
        self.current_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw

    def bno055_callback(self, msg):
        # Update angular orientations (assuming msg contains roll, pitch, yaw as angular.x, angular.y, angular.z)
        self.current_orientation[0] = msg.linear.x
        self.current_orientation[1] = msg.linear.y
        self.current_orientation[2] = msg.linear.z

        # Publish the updated transform
        self.publish_transform()

    def publish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.current_position[0]
        t.transform.translation.y = self.current_position[1]
        t.transform.translation.z = self.current_position[2]

        # Convert orientation from Euler angles to quaternion
        q = self.euler_to_quaternion(self.current_orientation[0], self.current_orientation[1], self.current_orientation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published transform at x={t.transform.translation.x}, y={t.transform.translation.y}, z={t.transform.translation.z}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    rclpy.spin(motor_controller_node)
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
