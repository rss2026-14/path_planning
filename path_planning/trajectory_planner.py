import rclpy

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from path_planning.utils import LineTrajectory
from rclpy.node import Node
import numpy as np

class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.pose_cb,
            10
        )

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.current_pose = None

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def pose_cb(self, pose):
        self.current_pose = (pose.pose.pose.position.x, pose.pose.pose.position.y)

    def goal_cb(self, msg):
        if self.map_data is None or self.current_pose is None:
            self.get_logger().warn("Waiting for map or pose")

        goal = (msg.pose.position.x, msg.pose.position.y)

        self.plan_path(self.current_pose, goal, self.map_data)

    def plan_path(self, start_point, end_point, map):
        path = self.rrt(start_point, end_point, map)

        if path is None:
            self.get_logger().warn("Failed to find a path")
            return
        
        self.trajectory.clear()

        for p in path:
            self.trajectory.addPoint(p[0], p[1])

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

        self.get_logger().info("Path published")

    def rrt(self, start_point, end_point, map):
        
        nodes = [start_point]


    


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
