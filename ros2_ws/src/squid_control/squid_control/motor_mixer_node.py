import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class MotorMixerNode(Node):
    """
    ROS 2 Node that translates velocity commands (Twist)
    into individual motor thrusts using a Mixer Matrix.
    """
    def __init__(self):
        super().__init__('motor_mixer')
        
        # Subscriber for flight commands (from pilot or AI)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        # Publisher for raw motor PWM values
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_thrusts', 10)
        
        # Define the 4x4 Mixer Matrix (Standard X-Frame)
        # Columns: [Throttle, Roll, Pitch, Yaw]
        # Rows: [M1, M2, M3, M4]
        self.mixer_matrix = np.array([
            [1.0, -1.0,  1.0, -1.0],  # Motor 1 (Front Left)
            [1.0,  1.0,  1.0,  1.0],  # Motor 2 (Front Right)
            [1.0, -1.0, -1.0,  1.0],  # Motor 3 (Rear Left)
            [1.0,  1.0, -1.0, -1.0]   # Motor 4 (Rear Right)
        ])

        self.get_logger().info('Squid Motor Mixer Node Started')

    def listener_callback(self, msg):
        # Extract commands from Twist message
        throttle = msg.linear.z
        roll = msg.angular.x
        pitch = msg.angular.y
        yaw = msg.angular.z

        # Create command vector
        u = np.array([throttle, roll, pitch, yaw])

        # Multiply by mixer matrix to get motor outputs
        motor_outputs = np.dot(self.mixer_matrix, u)

        # Normalize and Clip (0.0 to 1.0)
        motor_outputs = np.clip(motor_outputs, 0.0, 1.0)

        # Publish
        output_msg = Float32MultiArray()
        output_msg.data = motor_outputs.tolist()
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorMixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
