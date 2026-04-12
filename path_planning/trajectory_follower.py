import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from .utils import LineTrajectory

import numpy as np


class PurePursuit(Node):
    """
    Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 1.0         # 1.0 meter lookahead distance
        self.speed = 1.0             # 1.0 m/s constant speed
        self.wheelbase_length = 0.33 # 0.33 meters wheelbase

        self.initialized_traj = False
        self.trajectory = LineTrajectory(self, "/followed_trajectory")

        self.pose_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

    def pose_callback(self, odometry_msg):
        if not self.initialized_traj or self.trajectory.empty():
            return

        # 1. Extract current position and yaw from odometry
        pose = odometry_msg.pose.pose
        curr_x = pose.position.x
        curr_y = pose.position.y

        # Convert quaternion to euler yaw
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # 2. Find the lookahead point
        target_x, target_y = self._find_lookahead_point(curr_x, curr_y, yaw)

        # If we couldn't find a valid point, continue straight
        if target_x is None:
            self._publish_drive_command(self.speed, 0.0)
            return

        # 3. Transform the target point to the vehicle's local frame
        # Translate to origin, then rotate by the negative of the vehicle's yaw
        dx = target_x - curr_x
        dy = target_y - curr_y

        local_x = np.cos(yaw) * dx + np.sin(yaw) * dy
        local_y = -np.sin(yaw) * dx + np.cos(yaw) * dy

        # 4. Calculate steering angle
        actual_lookahead_sq = local_x**2 + local_y**2

        if actual_lookahead_sq > 0:
            # Pure pursuit formulation
            steering_angle = np.atan2(2.0 * self.wheelbase_length * local_y, actual_lookahead_sq)
        else:
            steering_angle = 0.0

        # Clip steering angle to physical limits (approx +/- 20 degrees or 0.34 rads)
        steering_angle = np.clip(steering_angle, -0.34, 0.34)

        # 5. Publish the drive command
        self._publish_drive_command(self.speed, float(steering_angle))


    def _find_lookahead_point(self, curr_x, curr_y, yaw):
        """
        Finds the point on the trajectory closest to the lookahead distance
        that is also IN FRONT of the vehicle.
        """
        best_pt = None
        closest_dist_diff = float('inf')

        # self.trajectory.points is a list of (x, y) tuples from utils.py
        for pt_x, pt_y in self.trajectory.points:

            # Vector from car to point
            dx = pt_x - curr_x
            dy = pt_y - curr_y

            # Check if point is in front of the car using the dot product
            # Car heading vector is (cos(yaw), sin(yaw))
            dot_product = (np.cos(yaw) * dx) + (np.sin(yaw) * dy)

            if dot_product > 0.0:  # strictly > 0 means it's in front
                dist = np.sqrt(dx**2 + dy**2)

                # Find the point whose distance from the car is closest to our lookahead radius
                dist_diff = abs(dist - self.lookahead)
                if dist_diff < closest_dist_diff:
                    closest_dist_diff = dist_diff
                    best_pt = (pt_x, pt_y)

        return best_pt

    def _publish_drive_command(self, speed, steering_angle):
        """ Helper function to construct and publish the Ackermann message. """
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_cmd)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
