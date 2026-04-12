import rclpy
import numpy as np

import cv2

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from path_planning.utils import LineTrajectory
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathPlan(Node):
    """ listens for goal pose published by rviz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")

        # declare ros parameters
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        # setup subscriptions and publishers
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)

        # setup tf2 for tracking coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # initialize empty state variables
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_yaw = 0.0  # store map rotation
        self.map_frame = None

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        # DILATE MAP
        raw_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        map_uint8 = np.uint8(raw_map)

        # calculate kernel size based on your car
        car_radius_meters = 0.5
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

    # helper math to handle map rotation seamlessly
    def world_to_grid(self, x, y):
        # translate relative to origin
        dx = x - self.map_origin[0]
        dy = y - self.map_origin[1]
        # un-rotate the coordinates
        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y
        # scale to map pixels
        u = int(lx / self.map_resolution)
        v = int(ly / self.map_resolution)
        return u, v

    def grid_to_world(self, u, v):
        # scale from pixels to meters
        lx = u * self.map_resolution
        ly = v * self.map_resolution
        # re-rotate back to world orientation
        cos_y = np.cos(self.map_yaw)
        sin_y = np.sin(self.map_yaw)
        dx = lx * cos_y - ly * sin_y
        dy = lx * sin_y + ly * cos_y
        # translate back to global position
        x = dx + self.map_origin[0]
        y = dy + self.map_origin[1]
        return x, y

    def goal_cb(self, msg):
        # make sure map is fully loaded before doing anything
        if self.map_data is None or self.map_frame is None:
            self.get_logger().warn("waiting for map data...")
            return

        try:
            # grab the car's real-time position mapped to the current map frame
            t = self.tf_buffer.lookup_transform(self.map_frame, 'base_link', rclpy.time.Time())
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except Exception as ex:
            self.get_logger().error(f"could not transform car pose: {ex}")
            return

        goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan_path(self.current_pose, goal, self.map_data)

    def plan_path(self, start_point, end_point, map_data):
        # convert world coordinates to grid indices
        start_u, start_v = self.world_to_grid(start_point[0], start_point[1])
        goal_u, goal_v = self.world_to_grid(end_point[0], end_point[1])

        height, width = map_data.shape

        self.get_logger().info("\n--- boundary debug ---")
        self.get_logger().info(f"map origin: {self.map_origin} | rotation: {np.degrees(self.map_yaw):.2f} deg")
        self.get_logger().info(f"start point (world): {start_point} --> grid (u,v): {start_u}, {start_v}")
        self.get_logger().info(f"goal point  (world): {end_point} --> grid (u,v): {goal_u}, {goal_v}")
        self.get_logger().info("----------------------\n")

        # safety check: ensure start and goal are physically inside the map
        if not (0 <= start_u < width and 0 <= start_v < height):
            self.get_logger().error(f"start point {start_point} is outside the map boundaries!")
            return
        if not (0 <= goal_u < width and 0 <= goal_v < height):
            self.get_logger().error(f"goal point {end_point} is outside the map boundaries!")
            return

        # safety check: ensure start and goal are not inside a wall
        if map_data[start_v, start_u] >= 50 or map_data[start_v, start_u] == -1:
            self.get_logger().error(f"start point is inside an obstacle!")
            return

        if map_data[goal_v, goal_u] >= 50 or map_data[goal_v, goal_u] == -1:
            self.get_logger().error(f"goal point is inside an obstacle!")
            return

        self.get_logger().info("planning path...")
        path = self.rrt_star(start_point, end_point, map_data)

        if path is None:
            self.get_logger().warn("failed to find a path")
            return

        # clear the old trajectory and add the newly planned points
        self.trajectory.clear()
        for p in path:
            self.trajectory.addPoint((p[0], p[1]))

        # publish out to rviz
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()
        self.get_logger().info("path published")

    def rrt_star(self, start, goal, map_data, max_iters=5000, delta=0.5, r=2.0, bridge_prob=0.5):
        # setup tree lists to store graph connections and costs
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

            # find the closest existing node in the tree to our random point
            diffs = tree_positions - x_rand
            dists = np.linalg.norm(diffs, axis=1)
            idx_nearest = np.argmin(dists)
            x_nearest = tree_positions[idx_nearest]

            # steer towards the random point by a maximum distance of delta
            vec = x_rand - x_nearest
            dist = np.linalg.norm(vec)
            x_new = x_nearest + delta * vec / dist if dist > delta else x_rand

            # check if the new line crosses an obstacle
            if not self.collision_free(x_nearest, x_new, map_data):
                continue

            # find other nearby nodes within radius r for rewiring
            vecs = tree_positions - x_new
            neighbor_dists = np.linalg.norm(vecs, axis=1)
            neighbors = np.where(neighbor_dists <= r)[0]

            # determine the most efficient parent node to connect to
            min_cost = tree_costs[idx_nearest] + np.linalg.norm(x_new - x_nearest)
            parent_idx = idx_nearest
            for n_idx in neighbors:
                cost_through_n = tree_costs[n_idx] + np.linalg.norm(x_new - tree_positions[n_idx])
                if self.collision_free(tree_positions[n_idx], x_new, map_data) and cost_through_n < min_cost:
                    min_cost = cost_through_n
                    parent_idx = n_idx

            # add the newly steered node to the tree
            tree_positions = np.vstack([tree_positions, x_new])
            tree_parents = np.append(tree_parents, parent_idx)
            tree_costs = np.append(tree_costs, min_cost)

            # optimize tree: check if existing neighbors get a cheaper path through our new node
            for n_idx in neighbors:
                cost_through_new = min_cost + np.linalg.norm(tree_positions[n_idx] - x_new)
                if self.collision_free(x_new, tree_positions[n_idx], map_data) and cost_through_new < tree_costs[n_idx]:
                    tree_parents[n_idx] = len(tree_positions) - 1
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

        # loop counter acts as an emergency brake against out-of-memory crashes
        loop_safety = 0
        while idx != -1 and loop_safety < max_iters + 10:
            path.append(tree_positions[idx])
            idx = tree_parents[idx]
            loop_safety += 1

        if loop_safety >= max_iters + 10:
            self.get_logger().error("emergency brake triggered: rrt graph contains a cycle!")
            return None

        # return the reversed path so it goes from start -> goal
        return path[::-1] if len(path) > 1 else None

    def bridge_sample(self, map_data, max_attempts=10):
        # heuristic to find narrow gaps by looking for space between two obstacles
        height, width = map_data.shape

        for _ in range(max_attempts):
            u1, v1 = np.random.uniform(0, width), np.random.uniform(0, height)
            u2, v2 = np.random.uniform(0, width), np.random.uniform(0, height)

            # check if both random points hit a wall
            if map_data[int(v1), int(u1)] > 50 and map_data[int(v2), int(u2)] > 50:
                x_mid, y_mid = self.grid_to_world((u1+u2)/2, (v1+v2)/2)
                u_mid, v_mid = self.world_to_grid(x_mid, y_mid)

                # if the midpoint between the walls is free, it's a good sample point
                if 0 <= map_data[v_mid, u_mid] < 50:
                    return np.array([x_mid, y_mid])

        # fallback to uniform sampling
        u_rand, v_rand = np.random.uniform(0, width), np.random.uniform(0, height)
        xr, yr = self.grid_to_world(u_rand, v_rand)
        return np.array([xr, yr])

    def collision_free(self, x1, x2, map_data):
        # discretize the line segment between two nodes
        vec = x2 - x1
        dist = np.linalg.norm(vec)
        n = max(int(dist / (self.map_resolution * 0.5)), 1)
        t = np.linspace(0, 1, n)
        xi = x1[0] + t * vec[0]
        yi = x1[1] + t * vec[1]

        # use our rotation-aware math to calculate the entire vector line on the grid
        dx = xi - self.map_origin[0]
        dy = yi - self.map_origin[1]
        cos_y = np.cos(-self.map_yaw)
        sin_y = np.sin(-self.map_yaw)
        lx = dx * cos_y - dy * sin_y
        ly = dx * sin_y + dy * cos_y

        u = (lx / self.map_resolution).astype(int)
        v = (ly / self.map_resolution).astype(int)

        # verify the line stays within map boundaries
        if np.any(u < 0) or np.any(u >= map_data.shape[1]) or np.any(v < 0) or np.any(v >= map_data.shape[0]):
            return False

        # ensure every pixel on the line is open space
        return np.all((map_data[v, u] >= 0) & (map_data[v, u] < 50))


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
