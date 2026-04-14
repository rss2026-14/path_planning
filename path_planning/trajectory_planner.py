import math
import heapq
import rclpy
import numpy as np

import cv2

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from path_planning.utils import LineTrajectory
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import SetParametersResult


class PathPlan(Node):
    """Listens for goal pose published by RViz and plans a path from
    the current car pose using either sampling-based or grid-based planning.
    """

    def __init__(self):
        super().__init__("trajectory_planner")

        # declare ros parameters
        self.declare_parameter("odom_topic", "default")
        self.declare_parameter("map_topic", "default")
        self.declare_parameter("planner_type", "sampling")   # "sampling" or "grid"
        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("inflate_radius", 0.43)

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.planner_type = self.get_parameter("planner_type").get_parameter_value().string_value
        self.occupancy_threshold = self.get_parameter("occupancy_threshold").get_parameter_value().integer_value
        self.inflate_radius = self.get_parameter("inflate_radius").get_parameter_value().float_value

        # setup subscriptions and publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 1
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, 10
        )
        self.traj_pub = self.create_publisher(
            PoseArray, "/trajectory/current", 10
        )

        # setup tf2 for tracking coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # initialize empty state variables
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_yaw = 0.0
        self.map_frame = None
        self.free_grid = None   # used by grid-based planning

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        # DILATE MAP
        raw_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        map_uint8 = np.uint8(raw_map)

        # calculate kernel size based on your car
        car_radius_meters = self.inflate_radius
        dilation_pixels = int(car_radius_meters / msg.info.resolution)

        # create the circular structuring element from your script
        kernel_size = 2 * dilation_pixels + 1
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        # apply the dilation and convert back to standard python ints for the collision checker
        dilated_map = cv2.dilate(map_uint8, element)
        self.map_data = dilated_map.astype(int)

        # save the rest of the metadata
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_frame = msg.header.frame_id

        q = msg.info.origin.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.map_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # build traversability grid for A*
        blocked = (self.map_data >= self.occupancy_threshold) | (self.map_data == -1)
        self.free_grid = ~blocked

        self.get_logger().info(
            f"Map received. planner_type={self.planner_type}, "
            f"inflate_radius={self.inflate_radius}"
        )

    def world_to_grid(self, x, y):
        dx = x - self.map_origin[0]
        dy = y - self.map_origin[1]

        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y

        u = int(lx / self.map_resolution)
        v = int(ly / self.map_resolution)
        return u, v

    def grid_to_world(self, u, v):
        lx = (u + 0.5) * self.map_resolution
        ly = (v + 0.5) * self.map_resolution

        cos_y = np.cos(self.map_yaw)
        sin_y = np.sin(self.map_yaw)
        dx = lx * cos_y - ly * sin_y
        dy = lx * sin_y + ly * cos_y

        x = dx + self.map_origin[0]
        y = dy + self.map_origin[1]
        return x, y

    def goal_cb(self, msg):
        if self.map_data is None or self.map_frame is None:
            self.get_logger().warn("waiting for map data...")
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, "base_link", rclpy.time.Time()
            )
            current_pose = (
                t.transform.translation.x,
                t.transform.translation.y,
            )
        except Exception as ex:
            self.get_logger().error(f"could not transform car pose: {ex}")
            return

        goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan_path(current_pose, goal, self.map_data)

    def plan_path(self, start_point, end_point, map_data):
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1])
        goal_u, goal_v = self.world_to_grid(end_point[0], end_point[1])

        height, width = map_data.shape

        if not (0 <= start_u < width and 0 <= start_v < height):
            self.get_logger().error(f"start point {start_point} is outside the map")
            return
        if not (0 <= goal_u < width and 0 <= goal_v < height):
            self.get_logger().error(f"goal point {end_point} is outside the map")
            return

        if self.planner_type == "sampling":
            if map_data[start_v, start_u] >= self.occupancy_threshold or map_data[start_v, start_u] == -1:
                self.get_logger().error("start point is inside an obstacle")
                return
            if map_data[goal_v, goal_u] >= self.occupancy_threshold or map_data[goal_v, goal_u] == -1:
                self.get_logger().error("goal point is inside an obstacle")
                return

            self.get_logger().info("planning with sampling-based RRT*...")
            path = self.rrt_star(start_point, end_point, map_data)

        elif self.planner_type == "grid":
            if not self.free_grid[start_v, start_u]:
                self.get_logger().error("start point is blocked in inflated grid")
                return
            if not self.free_grid[goal_v, goal_u]:
                self.get_logger().error("goal point is blocked in inflated grid")
                return

            self.get_logger().info("planning with grid-based A*...")
            path = self.a_star_grid((start_u, start_v), (goal_u, goal_v))

            if path is not None:
                # convert grid cells back to world coordinates
                path = [self.grid_to_world(u, v) for (u, v) in path]

        else:
            self.get_logger().error(
                f"Unknown planner_type '{self.planner_type}'. Use 'sampling' or 'grid'."
            )
            return

        if path is None:
            self.get_logger().warn("failed to find a path")
            return

        self.trajectory.clear()
        for p in path:
            # use whichever addPoint signature your LineTrajectory expects
            self.trajectory.addPoint((p[0], p[1]))
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()
        self.get_logger().info("path published")

    # grid-based planner
    def a_star_grid(self, start_cell, goal_cell):
        neighbors = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)),
            ( 1,  1, math.sqrt(2)),
        ]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start_cell))

        came_from = {}
        g_score = {start_cell: 0.0}
        visited = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in visited:
                continue
            visited.add(current)

            if current == goal_cell:
                return self.reconstruct_path(came_from, current)

            cu, cv = current

            for du, dv, move_cost in neighbors:
                nu, nv = cu + du, cv + dv

                if not self.in_bounds(nu, nv):
                    continue
                if not self.free_grid[nv, nu]:
                    continue

                # prevent diagonal corner-cutting
                if du != 0 and dv != 0:
                    if not self.free_grid[cv, cu + du]:
                        continue
                    if not self.free_grid[cv + dv, cu]:
                        continue

                neighbor = (nu, nv)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.grid_heuristic(neighbor, goal_cell)
                    heapq.heappush(open_heap, (f, neighbor))

        return None

    def grid_heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def in_bounds(self, u, v):
        h, w = self.map_data.shape
        return 0 <= u < w and 0 <= v < h


    # sampling-based planner
    def rrt_star(self, start, goal, map_data, max_iters=5000, delta=0.5, r=2.0, bridge_prob=0.5):
        tree_positions = np.array([start], dtype=float)
        tree_parents = np.array([-1], dtype=int)
        tree_costs = np.array([0.0], dtype=float)
        height, width = map_data.shape
        best_goal_idx = -1
        min_goal_cost = float('inf')

        for _ in range(max_iters):
            rand_float = np.random.rand()

            # 5% of the time, sample the exact goal to pull the tree towards it
            if rand_float < 0.05:
                x_rand = np.array([goal[0], goal[1]])
            # bridge sampling for narrow gaps
            elif rand_float < 0.05 + bridge_prob:
                x_rand = self.bridge_sample(map_data)
            # pure random sampling
            else:
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
            if dist < 1e-9:
                continue

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

            new_idx = len(tree_positions) - 1

            for n_idx in neighbors:
                cost_through_new = min_cost + np.linalg.norm(tree_positions[n_idx] - x_new)
                if self.collision_free(x_new, tree_positions[n_idx], map_data) and cost_through_new < tree_costs[n_idx]:
                    tree_parents[n_idx] = new_idx
                    tree_costs[n_idx] = cost_through_new
            # check if we are close enough to call the goal reached
            if np.linalg.norm(x_new - goal) < delta:
                cost_to_goal = min_cost + np.linalg.norm(x_new - goal)

                # If this is the shortest path to the goal we've found so far, save it!
                if cost_to_goal < min_goal_cost:
                    min_goal_cost = cost_to_goal
                    best_goal_idx = len(tree_positions) - 1 # save index of x_new

        # build the path
        if best_goal_idx == -1:
            self.get_logger().warn("max iterations reached without finding goal.")
            return None

        # officially add the goal to the tree using our best found parent
        tree_positions = np.vstack([tree_positions, goal])
        tree_parents = np.append(tree_parents, best_goal_idx)
        path = []
        idx = len(tree_positions) - 1
        loop_safety = 0

        while idx != -1 and loop_safety < max_iters + 10:
            path.append(tree_positions[idx])
            idx = tree_parents[idx]
            loop_safety += 1

        if loop_safety >= max_iters + 10:
            self.get_logger().error("emergency brake triggered: rrt graph contains a cycle!")
            return None

        return path[::-1] if len(path) > 1 else None

    def bridge_sample(self, map_data, max_attempts=10):
        height, width = map_data.shape

        for _ in range(max_attempts):
            u1, v1 = np.random.uniform(0, width), np.random.uniform(0, height)
            u2, v2 = np.random.uniform(0, width), np.random.uniform(0, height)

            if map_data[int(v1), int(u1)] > self.occupancy_threshold and map_data[int(v2), int(u2)] > self.occupancy_threshold:
                x_mid, y_mid = self.grid_to_world((u1 + u2) / 2, (v1 + v2) / 2)
                u_mid, v_mid = self.world_to_grid(x_mid, y_mid)

                if 0 <= u_mid < width and 0 <= v_mid < height:
                    if 0 <= map_data[v_mid, u_mid] < self.occupancy_threshold:
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

        dx = xi - self.map_origin[0]
        dy = yi - self.map_origin[1]
        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y

        u = (lx / self.map_resolution).astype(int)
        v = (ly / self.map_resolution).astype(int)

        if np.any(u < 0) or np.any(u >= map_data.shape[1]) or np.any(v < 0) or np.any(v >= map_data.shape[0]):
            return False

        return np.all((map_data[v, u] >= 0) & (map_data[v, u] < self.occupancy_threshold))


    def parameters_callback(self, params):
        """
        Dynamically updates parameters when modified via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'planner_type':
                self.planner_type = param.value
                self.get_logger().info(f"Updated planner type to {self.planner_type}")
            elif param.name == 'inflate_radius':
                self.inflate_radius = param.value
                self.get_logger().info(f"Updated inflate_radius to {self.inflate_radius}")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
