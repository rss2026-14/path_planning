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
        self.speed = 2.0             # 2.0 m/s constant speed
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

        # Extract current position and yaw from odometry
        pose = odometry_msg.pose.pose
        curr_x = pose.position.x
        curr_y = pose.position.y

        # Convert quaternion to euler yaw
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Are we theree yetttt
        goal_x, goal_y = self.trajectory.points[-1]
        dist_to_goal = np.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)

        # Use a small physical threshold (15cm).
        # A moving robot will never hit exactly 0.0 distance.
        if dist_to_goal < 0.15:
            self.get_logger().info("Goal Reached! Stopping car.")
            self._publish_drive_command(0.0, 0.0)
            self.trajectory.clear() # Wipe the path so we don't keep looping
            return

        target_pt = self._find_lookahead_point(curr_x, curr_y, yaw)

        if target_pt is None:
            # If the goal is inside our lookahead circle, just target the goal directly.
            if dist_to_goal <= self.lookahead:
                target_pt = (goal_x, goal_y)
            else:
                # If we are far away and lost the path, safely stop.
                self.get_logger().warn("No lookahead point found! Stopping.")
                self._publish_drive_command(0.0, 0.0)
                return

        target_x, target_y = target_pt

        # Transform the target point to the vehicle's local frame
        # Translate to origin, then rotate by the negative of the vehicle's yaw
        dx = target_x - curr_x
        dy = target_y - curr_y

        local_x = np.cos(yaw) * dx + np.sin(yaw) * dy
        local_y = -np.sin(yaw) * dx + np.cos(yaw) * dy

        # 4. Calculate steering angle
        actual_lookahead_sq = local_x**2 + local_y**2

        if actual_lookahead_sq > 0:
            # Pure pursuit formulation
            steering_angle = np.arctan2(2.0 * self.wheelbase_length * local_y, actual_lookahead_sq)
        else:
            steering_angle = 0.0

        # Clip steering angle to physical limits (approx +/- 20 degrees or 0.34 rads)
        steering_angle = np.clip(steering_angle, -0.34, 0.34)

        # 5. Publish the drive command
        self._publish_drive_command(self.speed, float(steering_angle))


    def _find_lookahead_point(self, curr_x, curr_y, yaw):
        """
        Finds the intersection between the lookahead circle and the piecewise
        continuous trajectory segments.
        """
        if len(self.trajectory.points) < 2:
            return None

        target_pt = None

        for i in range(len(self.trajectory.points) - 1):
            pt1 = self.trajectory.points[i]
            pt2 = self.trajectory.points[i+1]

            # Shift segment to origin based on car position
            v1 = np.array([pt1[0] - curr_x, pt1[1] - curr_y])
            v2 = np.array([pt2[0] - curr_x, pt2[1] - curr_y])

            # Direction vector of the segment
            d = v2 - v1

            # Quadratic coefficients for circle-line intersection
            a = np.dot(d, d)

            # FIX: Prevent divide-by-zero if the segment length is functionally zero
            if a < 1e-6:
                continue

            b = 2.0 * np.dot(v1, d)
            c = np.dot(v1, v1) - self.lookahead**2

            discriminant = b**2 - 4*a*c

            if discriminant >= 0:
                # Two possible solutions for t (where the line intersects the circle)
                t1 = (-b - np.sqrt(discriminant)) / (2.0 * a)
                t2 = (-b + np.sqrt(discriminant)) / (2.0 * a)

                # Check if the intersection points lie ON the segment (t must be between 0 and 1)
                valid_t = [t for t in (t1, t2) if 0.0 <= t <= 1.0]

                if valid_t:
                    # If multiple intersections on this segment, take the larger t (further forward)
                    best_t = max(valid_t)

                    intersection_x = pt1[0] + best_t * (pt2[0] - pt1[0])
                    intersection_y = pt1[1] + best_t * (pt2[1] - pt1[1])

                    # Verify it's actually in front of the car using dot product
                    dx = intersection_x - curr_x
                    dy = intersection_y - curr_y
                    if (np.cos(yaw) * dx + np.sin(yaw) * dy) > 0:
                        target_pt = (intersection_x, intersection_y)
                        # We don't break here! If the path loops or curves back into the circle,
                        # the later segments will overwrite this, keeping us moving strictly forward.

        return target_pt

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
