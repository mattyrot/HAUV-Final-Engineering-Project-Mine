import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty
import numpy as np

class ResetTransformNode(Node):
    def __init__(self):
        super().__init__('reset_transform_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0]
        
        # Create a service to reset the transform
        self.srv = self.create_service(Empty, 'reset_rov', self.reset_callback)
        
        # Timer to publish the transform regularly
        self.timer = self.create_timer(0.1, self.publish_transform)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    
    def publish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.current_position[0]
        t.transform.translation.y = self.current_position[1]
        t.transform.translation.z = self.current_position[2]

        q = self.euler_to_quaternion(self.current_orientation[0], self.current_orientation[1], self.current_orientation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published transform at x={self.current_position[0]}, y={self.current_position[1]}, z={self.current_position[2]}')

    def reset_callback(self, request, response):
        self.current_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0]
        self.get_logger().info('ROV location has been reset to the origin')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
