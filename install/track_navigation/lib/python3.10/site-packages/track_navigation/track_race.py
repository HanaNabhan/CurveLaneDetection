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
        self.lane_detection_range = 5.0  # Maximum range for lane detection
        self.min_points_for_fit = 10  # Minimum number of points needed for polynomial fitting
        self.poly_degree = 2  # Degree of polynomial for fitting
        self.max_steering_angle = 1.0  # Maximum steering angle in radians

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
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter valid points
        valid_mask = (ranges > 0.1) & (ranges < self.lane_detection_range)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        # Convert polar to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Separate points into left and right lanes
        left_points = np.array([(xi, yi) for xi, yi in zip(x, y) if yi > 0])
        right_points = np.array([(xi, yi) for xi, yi in zip(x, y) if yi < 0])

        # Fit polynomials to left and right lanes
        left_poly = self.fit_polynomial(left_points)
        right_poly = self.fit_polynomial(right_points)

        # Control vehicle based on fitted polynomials
        steering_angle, throttle = self.control_vehicle(left_poly, right_poly)

        # Publish commands
        self.steering_pub.publish(Float32(data=steering_angle))
        self.throttle_pub.publish(Float32(data=throttle))

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
        target_distance = 2.0  # Distance ahead to calculate lane center

        if right_poly:
            # Focus on the right side for steering
            steering_angle = math.atan2(right_poly(target_distance), target_distance)
            throttle = 0.1
        else:
            # Default behavior if no lanes are visible
            steering_angle = 0.0
            throttle = 0.1

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
