import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import math
import matplotlib.pyplot as plt

"""
LiDAR Lane Follower with Visualization
---------------------------------------
This ROS 2 node processes LiDAR data for lane following and visualizes the data in real-time.

Author: Hana Nabhan
"""

class LaneCurveDetection(Node):
    def __init__(self):
        super().__init__('lidar_lane_follower')

        # LiDAR subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            10
        )

        # Publishers for steering and throttle
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)

        # Lane detection parameters
        self.lane_detection_range = 0.0  # Maximum range for lane detection
        self.min_points_for_fit = 0.0  # Minimum number of points needed for polynomial fitting
        self.poly_degree = 0.0  # Degree of polynomial for fitting
        self.max_steering_angle = 0.0 # Maximum steering angle in radians

        # Visualization setup
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-self.lane_detection_range, self.lane_detection_range)
        self.ax.set_ylim(-self.lane_detection_range, self.lane_detection_range)
        self.ax.set_title("LiDAR Visualization")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        plt.ion()
        plt.show()

    def lidar_callback(self, msg: LaserScan):
        # Extract LiDAR data
        

        # Filter valid points
       

        # Convert polar to Cartesian coordinates
        

        # Separate points into left and right lanes
        
        # Fit polynomials to left and right lanes
        
        # Control vehicle based on fitted polynomials
        

        # Update visualization
        self.visualize_lidar(x, y, left_points, right_points)

    def fit_polynomial(self, points):
        """Fit a polynomial to the given points if enough points are available."""
        if points.shape[0] < self.min_points_for_fit:
            return None  # Not enough points for a fit
        x, y = points[:, 0], points[:, 1]
        return np.poly1d(np.polyfit(x, y, self.poly_degree))

    def control_vehicle(self, left_poly, right_poly):
        """Calculate steering angle and throttle based on lane polynomials."""
        target_distance = 0.0  # Distance ahead to calculate lane center

        if right_poly:
            # Focus on the right side for steering
            
        else:
            # Default behavior if no lanes are visible
            

        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        return steering_angle, throttle

    def visualize_lidar(self, x, y, left_points, right_points):
        """Visualize LiDAR points in real-time using matplotlib."""
        self.ax.clear()
        self.ax.set_xlim(-self.lane_detection_range, self.lane_detection_range)
        self.ax.set_ylim(-self.lane_detection_range, self.lane_detection_range)
        self.ax.scatter(x, y, c='blue', s=5, label="LiDAR Points")

        # Highlight left and right points
        if left_points.size > 0:
            self.ax.scatter(left_points[:, 0], left_points[:, 1], c='green', s=5, label="Left Lane Points")
        if right_points.size > 0:
            self.ax.scatter(right_points[:, 0], right_points[:, 1], c='red', s=5, label="Right Lane Points")

        self.ax.legend()
        self.ax.grid(True)
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    lidar_lane_follower = LaneCurveDetection()

    try:
        rclpy.spin(lidar_lane_follower)
    except KeyboardInterrupt:
        pass

    lidar_lane_follower.destroy_node()
    rclpy.shutdown()
    plt.close()


if __name__ == '__main__':
    main()
