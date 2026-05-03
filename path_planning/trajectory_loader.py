#!/usr/bin/env python3
import rclpy
import time

from geometry_msgs.msg import PoseArray
from path_planning.utils import LineTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
class LoadTrajectory(Node):
    """ Loads a trajectory from the file system and publishes it to a ROS topic.
    """

    def __init__(self):
        super().__init__("trajectory_loader")

        self.declare_parameter("trajectory", "default")
        self.path = self.get_parameter("trajectory").get_parameter_value().string_value

        # initialize and load the trajectory
        self.trajectory = LineTrajectory(self, "/loaded_trajectory")
        self.get_logger().info(f"Loading from {self.path}")
        self.trajectory.load(self.path)

        self.pub_topic = "/goal_pose"
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseArray, self.pub_topic, latched_qos)

        # need to wait a short period of time before publishing the first message
        time.sleep(0.5)

        # visualize the loaded trajectory
        self.trajectory.publish_viz()

        # send the trajectory
        self.publish_trajectory()

    def publish_trajectory(self):
        if self.trajectory.empty():
            self.get_logger().warn("Loaded trajectory is empty. No goal published.")
            return

        goal_x, goal_y = self.trajectory.points[-1]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(goal_x)
        goal_msg.pose.position.y = float(goal_y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Publishing section endpoint to /goal_pose: x={goal_x:.2f}, y={goal_y:.2f}"
        )

        self.goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    load_trajectory = LoadTrajectory()
    rclpy.spin(load_trajectory)
    load_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
