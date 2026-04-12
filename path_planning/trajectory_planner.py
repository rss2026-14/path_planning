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
        path = self.rrt_star(start_point, end_point, map)

        if path is None:
            self.get_logger().warn("Failed to find a path")
            return

        self.trajectory.clear()

        for p in path:
            self.trajectory.addPoint(p[0], p[1])

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

        self.get_logger().info("Path published")

    def rrt_star(self, start, goal, map_data, max_iters=3000, delta=0.5, r=2.0, bridge_prob=0.5):

            tree_positions = np.array([start], dtype=float)
            tree_parents = np.array([-1], dtype=int)
            tree_costs = np.array([0.0], dtype=float)
            height, width = map_data.shape

            for _ in range(max_iters):
                #sample
                if np.random.rand() < bridge_prob:
                    x_rand = self.bridge_sample(map_data)
                else:
                    x_rand = np.array([np.random.uniform(0, width*self.map_resolution),
                                    np.random.uniform(0, height*self.map_resolution)])

                #nearest node
                diffs = tree_positions - x_rand
                dists = np.linalg.norm(diffs, axis=1)
                idx_nearest = np.argmin(dists)
                x_nearest = tree_positions[idx_nearest]

                #steer
                vec = x_rand - x_nearest
                dist = np.linalg.norm(vec)
                x_new = x_nearest + delta * vec / dist if dist > delta else x_rand

                #collision check
                if not self.collision_free(x_nearest, x_new, map_data):
                    continue

                #neighbors
                vecs = tree_positions - x_new
                neighbor_dists = np.linalg.norm(vecs, axis=1)
                neighbors = np.where(neighbor_dists <= r)[0]

                #choose closest parent
                min_cost = tree_costs[idx_nearest] + np.linalg.norm(x_new - x_nearest)
                parent_idx = idx_nearest
                for n_idx in neighbors:
                    cost_through_n = tree_costs[n_idx] + np.linalg.norm(x_new - tree_positions[n_idx])
                    if self.collision_free(tree_positions[n_idx], x_new, map_data) and cost_through_n < min_cost:
                        min_cost = cost_through_n
                        parent_idx = n_idx

                #add the new node
                tree_positions = np.vstack([tree_positions, x_new])
                tree_parents = np.append(tree_parents, parent_idx)
                tree_costs = np.append(tree_costs, min_cost)

                #rewire neighbors
                for n_idx in neighbors:
                    cost_through_new = min_cost + np.linalg.norm(tree_positions[n_idx] - x_new)
                    if self.collision_free(x_new, tree_positions[n_idx], map_data) and cost_through_new < tree_costs[n_idx]:
                        tree_parents[n_idx] = len(tree_positions) - 1
                        tree_costs[n_idx] = cost_through_new

                #check goal
                if np.linalg.norm(x_new - goal) < delta:
                    tree_positions = np.vstack([tree_positions, goal])
                    tree_parents = np.append(tree_parents, len(tree_positions)-1)
                    tree_costs = np.append(tree_costs, min_cost + np.linalg.norm(x_new - goal))
                    break

            path = []
            idx = len(tree_positions) - 1
            while idx != -1:
                path.append(tree_positions[idx])
                idx = tree_parents[idx]
            return path[::-1] if path else None

    def bridge_sample(self, map_data, max_attempts=10):

        height, width = map_data.shape
        for _ in range(max_attempts):
            x1, y1 = np.random.uniform(0, width*self.map_resolution), np.random.uniform(0, height*self.map_resolution)
            x2, y2 = np.random.uniform(0, width*self.map_resolution), np.random.uniform(0, height*self.map_resolution)
            u1, v1 = int(x1 / self.map_resolution), int(y1 / self.map_resolution)
            u2, v2 = int(x2 / self.map_resolution), int(y2 / self.map_resolution)

            if map_data[v1, u1] > 50 and map_data[v2, u2] > 50:
                x_mid = (x1 + x2)/2
                y_mid = (y1 + y2)/2
                u_mid, v_mid = int(x_mid / self.map_resolution), int(y_mid / self.map_resolution)
                if map_data[v_mid, u_mid] == 0:
                    return np.array([x_mid, y_mid])

        # uniform if bridge doesn't work
        return np.array([np.random.uniform(0, width*self.map_resolution),
                            np.random.uniform(0, height*self.map_resolution)])

    def collision_free(self, x1, x2, map_data):
        vec = x2 - x1
        dist = np.linalg.norm(vec)
        n = max(int(dist / (self.map_resolution * 0.5)), 1)
        t = np.linspace(0, 1, n)
        xi = x1[0] + t * vec[0]
        yi = x1[1] + t * vec[1]

        u = np.clip((xi / self.map_resolution).astype(int), 0, map_data.shape[1]-1)
        v = np.clip((yi / self.map_resolution).astype(int), 0, map_data.shape[0]-1)
        return np.all(map_data[v, u] == 0)

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
