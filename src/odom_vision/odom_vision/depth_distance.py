import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np

class DepthDistanceNode(Node):
    def __init__(self):
        super().__init__('depth_distance_node')
        
        # Subscribe to the depth topic
        # Use 'Best Effort' QoS if using a live camera feed for performance
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            10)
        
        self.get_logger().info('Depth Distance Node has started, listening to /depth...')
        
        self.pub = self.create_publisher(
            Float32,
            '/depth_distance',
            10
        )
            
            

    def depth_callback(self, msg):
        """
        Callback function to process a 32FC1 depth image message.
        """
        try:
            # Convert the ROS Image message to a NumPy array
            # msg.data is a byte array; we view it as 32-bit floats
            depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            
            # Get the distance at the center of the image
            center_x = msg.width // 2
            center_y = msg.height // 2
            distance = depth_array[center_y, center_x]
            
            # Handle NaN or Inf values (areas too close or too far for the sensor)
            if np.isnan(distance) or np.isinf(distance):
                self.get_logger().warn('Distance at center is NaN or Inf (out of range).', throttle_duration_sec=1.0)
                return
                
            # Log the metric distance in meters
            self.get_logger().info(f'Distance to center: {distance:.2f} meters', throttle_duration_sec=0.5)
            # Publish the distance as a Float32 message
            distance_msg = Float32()
            distance_msg.data = float(distance)
            self.pub.publish(distance_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()