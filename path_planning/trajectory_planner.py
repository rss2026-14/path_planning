# import rclpy

# from geometry_msgs.msg import PoseArray, PoseStamped
# from nav_msgs.msg import OccupancyGrid, Odometry
# from path_planning.utils import LineTrajectory
# from rclpy.node import Node
# import numpy as np

# class PathPlan(Node):
#     """ Listens for goal pose published by RViz and uses it to plan a path from
#     current car pose.
#     """

#     def __init__(self):
#         super().__init__("trajectory_planner")
#         self.declare_parameter('odom_topic', "default")
#         self.declare_parameter('map_topic', "default")

#         self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
#         self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             self.map_topic,
#             self.map_cb,
#             1)

#         self.goal_sub = self.create_subscription(
#             PoseStamped,
#             "/goal_pose",
#             self.goal_cb,
#             10
#         )

#         self.traj_pub = self.create_publisher(
#             PoseArray,
#             "/trajectory/current",
#             10
#         )

#         self.pose_sub = self.create_subscription(
#             Odometry,
#             self.odom_topic,
#             self.pose_cb,
#             10
#         )

#         self.map_data = None
#         self.map_resolution = None
#         self.map_origin = None
#         self.current_pose = None

#         self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

#     def map_cb(self, msg):
#         self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
#         self.map_resolution = msg.info.resolution
#         self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

#     def pose_cb(self, pose):
#         self.current_pose = (pose.pose.pose.position.x, pose.pose.pose.position.y)

#     def goal_cb(self, msg):
#         if self.map_data is None or self.current_pose is None:
#             self.get_logger().warn("Waiting for map or pose")
#             return

#         goal = (msg.pose.position.x, msg.pose.position.y)

#         self.plan_path(self.current_pose, goal, self.map_data)

#     def plan_path(self, start_point, end_point, map):
#         path = self.rrt_star(start_point, end_point, map)

#         if path is None:
#             self.get_logger().warn("Failed to find a path")
#             return

#         self.trajectory.clear()

#         for p in path:
#             self.trajectory.addPoint((p[0], p[1]))

#         self.traj_pub.publish(self.trajectory.toPoseArray())
#         self.trajectory.publish_viz()

#         self.get_logger().info("Path published")

#     def rrt_star(self, start, goal, map_data, max_iters=3000, delta=0.5, r=2.0, bridge_prob=0.5):

#             tree_positions = np.array([start], dtype=float)
#             tree_parents = np.array([-1], dtype=int)
#             tree_costs = np.array([0.0], dtype=float)
#             height, width = map_data.shape
#             # Calculate the true world boundaries of the map
#             x_min, y_min = self.map_origin
#             x_max = x_min + width * self.map_resolution
#             y_max = y_min + height * self.map_resolution

#             for _ in range(max_iters):
#                 #sample
#                 if np.random.rand() < bridge_prob:
#                     x_rand = self.bridge_sample(map_data)
#                 else:
#                     # SAMPLE USING THE ORIGIN OFFSETS
#                     x_rand = np.array([np.random.uniform(x_min, x_max),
#                                        np.random.uniform(y_min, y_max)])

#                 #nearest node
#                 diffs = tree_positions - x_rand
#                 dists = np.linalg.norm(diffs, axis=1)
#                 idx_nearest = np.argmin(dists)
#                 x_nearest = tree_positions[idx_nearest]

#                 #steer
#                 vec = x_rand - x_nearest
#                 dist = np.linalg.norm(vec)
#                 x_new = x_nearest + delta * vec / dist if dist > delta else x_rand

#                 #collision check
#                 if not self.collision_free(x_nearest, x_new, map_data):
#                     continue

#                 #neighbors
#                 vecs = tree_positions - x_new
#                 neighbor_dists = np.linalg.norm(vecs, axis=1)
#                 neighbors = np.where(neighbor_dists <= r)[0]

#                 #choose closest parent
#                 min_cost = tree_costs[idx_nearest] + np.linalg.norm(x_new - x_nearest)
#                 parent_idx = idx_nearest
#                 for n_idx in neighbors:
#                     cost_through_n = tree_costs[n_idx] + np.linalg.norm(x_new - tree_positions[n_idx])
#                     if self.collision_free(tree_positions[n_idx], x_new, map_data) and cost_through_n < min_cost:
#                         min_cost = cost_through_n
#                         parent_idx = n_idx

#                 #add the new node
#                 tree_positions = np.vstack([tree_positions, x_new])
#                 tree_parents = np.append(tree_parents, parent_idx)
#                 tree_costs = np.append(tree_costs, min_cost)

#                 #rewire neighbors
#                 for n_idx in neighbors:
#                     cost_through_new = min_cost + np.linalg.norm(tree_positions[n_idx] - x_new)
#                     if self.collision_free(x_new, tree_positions[n_idx], map_data) and cost_through_new < tree_costs[n_idx]:
#                         tree_parents[n_idx] = len(tree_positions) - 1
#                         tree_costs[n_idx] = cost_through_new

#                 #check goal
#                 if np.linalg.norm(x_new - goal) < delta:
#                     tree_positions = np.vstack([tree_positions, goal])
#                     tree_parents = np.append(tree_parents, len(tree_positions)-1)
#                     tree_costs = np.append(tree_costs, min_cost + np.linalg.norm(x_new - goal))
#                     break

#             path = []
#             idx = len(tree_positions) - 1
#             while idx != -1:
#                 path.append(tree_positions[idx])
#                 idx = tree_parents[idx]
#             return path[::-1] if path else None

#     def bridge_sample(self, map_data, max_attempts=10):
#         height, width = map_data.shape
#         x_min, y_min = self.map_origin
#         x_max = x_min + width * self.map_resolution
#         y_max = y_min + height * self.map_resolution

#         for _ in range(max_attempts):
#             x1, y1 = np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max)
#             x2, y2 = np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max)

#             # SUBTRACT ORIGIN BEFORE DIVIDING BY RESOLUTION
#             u1, v1 = int((x1 - x_min) / self.map_resolution), int((y1 - y_min) / self.map_resolution)
#             u2, v2 = int((x2 - x_min) / self.map_resolution), int((y2 - y_min) / self.map_resolution)

#             if map_data[v1, u1] > 50 and map_data[v2, u2] > 50:
#                 x_mid = (x1 + x2)/2
#                 y_mid = (y1 + y2)/2
#                 u_mid, v_mid = int((x_mid - x_min) / self.map_resolution), int((y_mid - y_min) / self.map_resolution)
#                 if map_data[v_mid, u_mid] == 0:
#                     return np.array([x_mid, y_mid])

#         # uniform if bridge doesn't work
#         return np.array([np.random.uniform(x_min, x_max),
#                          np.random.uniform(y_min, y_max)])

#     def collision_free(self, x1, x2, map_data):
#         vec = x2 - x1
#         dist = np.linalg.norm(vec)
#         n = max(int(dist / (self.map_resolution * 0.5)), 1)
#         t = np.linspace(0, 1, n)
#         xi = x1[0] + t * vec[0]
#         yi = x1[1] + t * vec[1]

#         # SUBTRACT ORIGIN BEFORE DIVIDING BY RESOLUTION
#         u = np.clip(((xi - self.map_origin[0]) / self.map_resolution).astype(int), 0, map_data.shape[1]-1)
#         v = np.clip(((yi - self.map_origin[1]) / self.map_resolution).astype(int), 0, map_data.shape[0]-1)

#         return np.all(map_data[v, u] == 0)

# def main(args=None):
#     rclpy.init(args=args)
#     planner = PathPlan()
#     rclpy.spin(planner)
#     rclpy.shutdown()

import rclpy
import numpy as np

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from path_planning.utils import LineTrajectory
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathPlan(Node):
    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_yaw = 0.0  # NEW: Store map rotation
        self.map_frame = None

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_frame = msg.header.frame_id

        # NEW: Extract the Yaw (rotation) from the map's origin quaternion
        q = msg.info.origin.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.map_yaw = np.arctan2(siny_cosp, cosy_cosp)

    # --- NEW: Helper math to handle map rotation seamlessly ---
    def world_to_grid(self, x, y):
        # 1. Translate
        dx = x - self.map_origin[0]
        dy = y - self.map_origin[1]
        # 2. Un-Rotate
        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y
        # 3. Scale to pixels
        u = int(lx / self.map_resolution)
        v = int(ly / self.map_resolution)
        return u, v

    def grid_to_world(self, u, v):
        # 1. Scale to meters
        lx = u * self.map_resolution
        ly = v * self.map_resolution
        # 2. Re-Rotate
        cos_y = np.cos(self.map_yaw)
        sin_y = np.sin(self.map_yaw)
        dx = lx * cos_y - ly * sin_y
        dy = lx * sin_y + ly * cos_y
        # 3. Translate
        x = dx + self.map_origin[0]
        y = dy + self.map_origin[1]
        return x, y
    # ---------------------------------------------------------

    def goal_cb(self, msg):
        if self.map_data is None or self.map_frame is None:
            self.get_logger().warn("Waiting for map data...")
            return

        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, 'base_link', rclpy.time.Time())
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except Exception as ex:
            self.get_logger().error(f"Could not transform car pose: {ex}")
            return

        goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan_path(self.current_pose, goal, self.map_data)

    def plan_path(self, start_point, end_point, map_data):
        # Use our new rotation-aware helper!
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1])
        goal_u, goal_v = self.world_to_grid(end_point[0], end_point[1])

        height, width = map_data.shape

        self.get_logger().info("\n--- BOUNDARY DEBUG ---")
        self.get_logger().info(f"Map Origin: {self.map_origin} | Rotation: {np.degrees(self.map_yaw):.2f} deg")
        self.get_logger().info(f"Start Point (World): {start_point} --> Grid (U,V): {start_u}, {start_v}")
        self.get_logger().info(f"Goal Point  (World): {end_point} --> Grid (U,V): {goal_u}, {goal_v}")
        self.get_logger().info("----------------------\n")

        if not (0 <= start_u < width and 0 <= start_v < height):
            self.get_logger().error(f"START point {start_point} is outside the map boundaries!")
            return
        if not (0 <= goal_u < width and 0 <= goal_v < height):
            self.get_logger().error(f"GOAL point {end_point} is outside the map boundaries!")
            return

        if map_data[start_v, start_u] >= 50 or map_data[start_v, start_u] == -1:
            self.get_logger().error(f"START point is inside an obstacle!")
            return

        if map_data[goal_v, goal_u] >= 50 or map_data[goal_v, goal_u] == -1:
            self.get_logger().error(f"GOAL point is inside an obstacle!")
            return

        self.get_logger().info("Planning path...")
        path = self.rrt_star(start_point, end_point, map_data)

        if path is None:
            self.get_logger().warn("Failed to find a path")
            return

        self.trajectory.clear()
        for p in path:
            self.trajectory.addPoint((p[0], p[1]))

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()
        self.get_logger().info("Path published")

    def rrt_star(self, start, goal, map_data, max_iters=3000, delta=0.5, r=2.0, bridge_prob=0.5):
        tree_positions = np.array([start], dtype=float)
        tree_parents = np.array([-1], dtype=int)
        tree_costs = np.array([0.0], dtype=float)
        height, width = map_data.shape

        for _ in range(max_iters):
            if np.random.rand() < bridge_prob:
                x_rand = self.bridge_sample(map_data)
            else:
                # Sample inside the grid pixels, then perfectly map it to real-world coordinates!
                u_rand = np.random.uniform(0, width)
                v_rand = np.random.uniform(0, height)
                xr, yr = self.grid_to_world(u_rand, v_rand)
                x_rand = np.array([xr, yr])

            diffs = tree_positions - x_rand
            dists = np.linalg.norm(diffs, axis=1)
            idx_nearest = np.argmin(dists)
            x_nearest = tree_positions[idx_nearest]

            vec = x_rand - x_nearest
            dist = np.linalg.norm(vec)
            x_new = x_nearest + delta * vec / dist if dist > delta else x_rand

            if not self.collision_free(x_nearest, x_new, map_data):
                continue

            vecs = tree_positions - x_new
            neighbor_dists = np.linalg.norm(vecs, axis=1)
            neighbors = np.where(neighbor_dists <= r)[0]

            min_cost = tree_costs[idx_nearest] + np.linalg.norm(x_new - x_nearest)
            parent_idx = idx_nearest
            for n_idx in neighbors:
                cost_through_n = tree_costs[n_idx] + np.linalg.norm(x_new - tree_positions[n_idx])
                if self.collision_free(tree_positions[n_idx], x_new, map_data) and cost_through_n < min_cost:
                    min_cost = cost_through_n
                    parent_idx = n_idx

            tree_positions = np.vstack([tree_positions, x_new])
            tree_parents = np.append(tree_parents, parent_idx)
            tree_costs = np.append(tree_costs, min_cost)

            for n_idx in neighbors:
                cost_through_new = min_cost + np.linalg.norm(tree_positions[n_idx] - x_new)
                if self.collision_free(x_new, tree_positions[n_idx], map_data) and cost_through_new < tree_costs[n_idx]:
                    tree_parents[n_idx] = len(tree_positions) - 1
                    tree_costs[n_idx] = cost_through_new

            # check goal
            if np.linalg.norm(x_new - goal) < delta:
                # FIX: Save the index of x_new BEFORE we add the goal to the array
                parent_idx_for_goal = len(tree_positions) - 1

                tree_positions = np.vstack([tree_positions, goal])
                tree_parents = np.append(tree_parents, parent_idx_for_goal) # Use the saved index
                tree_costs = np.append(tree_costs, min_cost + np.linalg.norm(x_new - goal))
                break

        path = []
        idx = len(tree_positions) - 1

        # FIX: Added a loop counter to act as an emergency brake against OOM crashes
        loop_safety = 0
        while idx != -1 and loop_safety < max_iters + 10:
            path.append(tree_positions[idx])
            idx = tree_parents[idx]
            loop_safety += 1

        if loop_safety >= max_iters + 10:
            self.get_logger().error("Emergency brake triggered: RRT graph contains a cycle!")
            return None

        return path[::-1] if len(path) > 1 else None

    def bridge_sample(self, map_data, max_attempts=10):
        height, width = map_data.shape

        for _ in range(max_attempts):
            u1, v1 = np.random.uniform(0, width), np.random.uniform(0, height)
            u2, v2 = np.random.uniform(0, width), np.random.uniform(0, height)

            if map_data[int(v1), int(u1)] > 50 and map_data[int(v2), int(u2)] > 50:
                x_mid, y_mid = self.grid_to_world((u1+u2)/2, (v1+v2)/2)
                u_mid, v_mid = self.world_to_grid(x_mid, y_mid)

                if 0 <= map_data[v_mid, u_mid] < 50:
                    return np.array([x_mid, y_mid])

        u_rand, v_rand = np.random.uniform(0, width), np.random.uniform(0, height)
        xr, yr = self.grid_to_world(u_rand, v_rand)
        return np.array([xr, yr])

    def collision_free(self, x1, x2, map_data):
        vec = x2 - x1
        dist = np.linalg.norm(vec)
        n = max(int(dist / (self.map_resolution * 0.5)), 1)
        t = np.linspace(0, 1, n)
        xi = x1[0] + t * vec[0]
        yi = x1[1] + t * vec[1]

        # Use our rotation-aware math to calculate the entire vector line
        dx = xi - self.map_origin[0]
        dy = yi - self.map_origin[1]
        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y

        u = (lx / self.map_resolution).astype(int)
        v = (ly / self.map_resolution).astype(int)

        # Check bounds
        if np.any(u < 0) or np.any(u >= map_data.shape[1]) or np.any(v < 0) or np.any(v >= map_data.shape[0]):
            return False

        return np.all((map_data[v, u] >= 0) & (map_data[v, u] < 50))


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
