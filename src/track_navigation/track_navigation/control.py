import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Proportional gain for steering
        self.kp_steering = 1.07  # Adjust the proportional gain to tune responsiveness
        self.max_steering_angle = 0.79  # Max allowable steering angle

        # Throttle control
        self.default_throttle_value = 0.04  # Default throttle value
        self.throttle_value = self.default_throttle_value  # Throttle will change dynamically

        # Create publishers
        self.steering_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)

        # Subscription to LIDAR topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            40
        )

        self.lidar_ranges = []
        self.get_logger().info('Follow The Gap Node has been started.')

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges
        self.control_loop()  # Call control loop directly after receiving LIDAR data

    def preprocess_lidar(self, lidar_ranges):
        # Replace inf values with the max sensor range (e.g., 8.0m)
        max_range = 5.0
        lidar_ranges = np.array(lidar_ranges)
        lidar_ranges[np.isinf(lidar_ranges)] = max_range
        return lidar_ranges

    def find_max_gap(self, lidar_ranges):
        # Identify gaps in the LIDAR scan (where no obstacles are present)
        min_distance_threshold = 1.2  # Threshold distance to consider an obstacle
        gap_starts = np.where(lidar_ranges > min_distance_threshold)[0]
        
        # Find the largest continuous gap
        if gap_starts.size == 0:  # No gaps found
            return np.array([])

        gaps = np.split(gap_starts, np.where(np.diff(gap_starts) > 1)[0] + 1)
        largest_gap = max(gaps, key=len)
        
        return largest_gap

    def find_best_point(self, gap, lidar_ranges):
        # Select the middle point of the largest gap
        best_point_idx = gap[len(gap) // 2]
        return best_point_idx

    def set_bubble(self, ranges, closest_point_idx, rb=0.6):
           """Apply a bubble around the closest point, clearing points within this bubble."""
           # Get the angle of the closest point
           angle = self.angles[closest_point_idx]
           # Calculate the angular width of the bubble in the LiDAR scan
           dtheta = np.arctan2(rb, ranges[closest_point_idx])
        
           # Find indices of angles within the bubble (near the closest point)
           bubble_idx = np.where(np.logical_and(self.angles > angle - dtheta, self.angles < angle + dtheta))
        
           # Set the values within the bubble to zero (indicating no obstacle)
           ranges[bubble_idx] = 0
        
           # Optionally, you can extend this to apply the bubble in a larger area around the closest point
        
           return ranges
        
    def control_loop(self):
       if not self.lidar_ranges:
           return

       # Preprocess the LIDAR data
       lidar_ranges = self.preprocess_lidar(self.lidar_ranges)

       # Define direction indices for front, left, and right
       num_ranges = len(lidar_ranges)
       front_start_idx = int(num_ranges * 0.4)   # 40% to 60% for front direction
       front_end_idx = int(num_ranges * 0.6)

       left_start_idx = int(num_ranges * 0.1)    # 10% to 40% for left direction
       left_end_idx = front_start_idx

       right_start_idx = front_end_idx           # 60% to 90% for right direction
       right_end_idx = int(num_ranges * 0.9)

       # Extract segments for each direction
       front_segment = lidar_ranges[front_start_idx:front_end_idx]
       left_segment = lidar_ranges[left_start_idx:left_end_idx]
       right_segment = lidar_ranges[right_start_idx:right_end_idx]

       # Find the largest gaps in each direction
       largest_front_gap = self.find_max_gap(front_segment)
       largest_left_gap = self.find_max_gap(left_segment)
       largest_right_gap = self.find_max_gap(right_segment)

       # Define emergency threshold (distance within which to trigger emergency steer)
       emergency_threshold = 0.55  # Meters, adjust this as needed for your system

       # Check if there's an obstacle too close in any direction
       emergency_steer = None
       if np.any(front_segment < emergency_threshold - 0.3):  # Front has a very close obstacle
           emergency_steer = "front"
       elif np.any(left_segment < emergency_threshold):  # Left has a very close obstacle
           emergency_steer = "left"
       elif np.any(right_segment < emergency_threshold):  # Right has a very close obstacle
           emergency_steer = "right"

       # Default gap_direction (if no gap is found, this avoids uninitialized variable)
       gap_direction = ""

       # If there's an emergency steer situation, override normal steering
       if emergency_steer:
           self.get_logger().warn(f"Emergency steer required in the {emergency_steer} direction!")
           # Apply maximum steering angle in the opposite direction to avoid collision
           if emergency_steer == "front":
                left_gap_size = len(largest_left_gap)
                right_gap_size = len(largest_right_gap)

                # Determine the direction with the larger gap
                if left_gap_size > right_gap_size:
                    steering_control = 1.0  # Steer right (to avoid the front obstacle)
                    gap_direction = "left"
                else:
                    steering_control = -1.0  # Steer left
                    gap_direction = "right"
           elif emergency_steer == "left":
               steering_control = 1.0 # Steer right
           elif emergency_steer == "right":
               steering_control = -1.0 # Steer left
       else:
           # No emergency, continue with regular gap following
           # Determine the best gap to follow based on distance or size
           best_gap = None
           best_distance = -1

           if largest_front_gap.size > 0:
               front_gap_distance = np.mean(front_segment[largest_front_gap])  # Use mean distance as a metric
               if front_gap_distance > best_distance:
                   best_distance = front_gap_distance
                   best_gap = largest_front_gap
                   gap_direction = "front"

           if largest_left_gap.size > 0:
               left_gap_distance = np.mean(left_segment[largest_left_gap])
               if left_gap_distance > best_distance:
                   best_distance = left_gap_distance
                   best_gap = largest_left_gap
                   gap_direction = "left"

           if largest_right_gap.size > 0:
               right_gap_distance = np.mean(right_segment[largest_right_gap])
               if right_gap_distance > best_distance:
                   best_distance = right_gap_distance
                   best_gap = largest_right_gap
                   gap_direction = "right"

           if best_gap is None:
               self.get_logger().warn('No valid gaps found.')
               return

           # Find the best point to aim for (middle of the selected best gap)
           best_point_idx = self.find_best_point(best_gap, lidar_ranges)

           # Calculate the error as the difference between the best point and the center of the LIDAR scan
           center_idx = num_ranges // 2
           error = (best_point_idx + (front_start_idx if gap_direction == "front" else left_start_idx if gap_direction == "left" else right_start_idx) - center_idx)

           # Steering control using Proportional-only control
           steering_control = self.kp_steering * error
           steering_control = np.clip(steering_control, -self.max_steering_angle, self.max_steering_angle)

       # Publish the steering control message
       steering_msg = Float32()
       steering_msg.data = steering_control
       self.steering_publisher.publish(steering_msg)

       # Throttle control (adjust based on steering angle)
       if abs(steering_control) > 0.6:  # Adjust this threshold as needed
           # Slow down if steering is significant
           self.throttle_value = self.default_throttle_value
       else:
           # Speed up if steering is minimal
           self.throttle_value = 0.19  # Increase speed (capped based on your system)

       throttle_msg = Float32()
       throttle_msg.data = self.throttle_value
       self.throttle_publisher.publish(throttle_msg)

       self.get_logger().info(f'Gap found in the {gap_direction} direction. Steering: {steering_control:.2f}, Throttle: {self.throttle_value:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
