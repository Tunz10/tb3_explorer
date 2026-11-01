import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat_w(yaw: float):
    # For 2D a simple (z, w) is enough; roll/pitch are assumed to be 0
    # yaw -> (z, w)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TB3FrontierExplorer(Node):
    def __init__(self):
        super().__init__('tb3_frontier_explorer')

        # ---- Parameters ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('min_frontier_cluster', 30)   # cells
        self.declare_parameter('goal_cooldown_sec', 2.0)
        self.declare_parameter('stuck_timeout_sec', 40.0)    # retry if a goal doesn't finish
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('occupied_threshold', 50)     # 50+ counts as occupied
        self.declare_parameter('max_goals', 999999)

        self.map_topic = self.get_parameter('map_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.min_cluster = int(self.get_parameter('min_frontier_cluster').value)
        self.goal_cooldown = float(self.get_parameter('goal_cooldown_sec').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout_sec').value)
        self.unknown_v = int(self.get_parameter('unknown_value').value)
        self.free_v = int(self.get_parameter('free_value').value)
        self.occ_th = int(self.get_parameter('occupied_threshold').value)
        self.max_goals = int(self.get_parameter('max_goals').value)

        # ---- State ----
        self.map_msg: OccupancyGrid | None = None
        self.last_goal_time = 0.0
        self.goal_sent_time = 0.0
        self.goals_sent = 0
        self.active_goal_handle = None

        # ---- ROS I/O ----
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # periodic control loop
        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info("TB3 Frontier Explorer ready. Waiting for /map...")

    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _tick(self):
        if self.map_msg is None:
            return

        now = time.time()

        # if a goal is active, check the stuck timeout
        if self.active_goal_handle is not None:
            if (now - self.goal_sent_time) > self.stuck_timeout:
                self.get_logger().warn("Goal timeout (stuck). Choosing a new frontier.")
                try:
                    self.active_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.active_goal_handle = None
            return

        # cooldown
        if (now - self.last_goal_time) < self.goal_cooldown:
            return

        if self.goals_sent >= self.max_goals:
            self.get_logger().info("max_goals reached. Explorer stopping.")
            rclpy.shutdown()
            return

        # find a frontier and send a goal
        goal = self._choose_frontier_goal(self.map_msg)
        if goal is None:
            self.get_logger().info("No frontier found. Mapping may be complete.")
            return

        gx, gy, yaw = goal
        self._send_goal(gx, gy, yaw)

    # -------- Frontier logic --------
    def _choose_frontier_goal(self, grid: OccupancyGrid):
        w = grid.info.width
        h = grid.info.height
        data = grid.data
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        def idx(x, y): return y * w + x

        # frontier cell: free, with at least one unknown neighbour
        def is_frontier(x, y):
            v = data[idx(x, y)]
            if v != self.free_v:
                return False
            for ny in range(max(0, y - 1), min(h, y + 2)):
                for nx in range(max(0, x - 1), min(w, x + 2)):
                    if nx == x and ny == y:
                        continue
                    if data[idx(nx, ny)] == self.unknown_v:
                        return True
            return False

        visited = [False] * (w * h)
        clusters = []

        for y in range(h):
            for x in range(w):
                i = idx(x, y)
                if visited[i]:
                    continue
                if not is_frontier(x, y):
                    continue

                # BFS cluster
                q = deque([(x, y)])
                visited[i] = True
                cells = []

                while q:
                    cx, cy = q.popleft()
                    cells.append((cx, cy))
                    for ny in range(max(0, cy - 1), min(h, cy + 2)):
                        for nx in range(max(0, cx - 1), min(w, cx + 2)):
                            ni = idx(nx, ny)
                            if visited[ni]:
                                continue
                            if is_frontier(nx, ny):
                                visited[ni] = True
                                q.append((nx, ny))

                if len(cells) >= self.min_cluster:
                    clusters.append(cells)

        if not clusters:
            return None

        # Simple choice: centroid of the largest cluster
        clusters.sort(key=lambda c: len(c), reverse=True)
        best = clusters[0]

        cx = sum(p[0] for p in best) / len(best)
        cy = sum(p[1] for p in best) / len(best)

        # grid -> world
        wx = origin_x + (cx + 0.5) * res
        wy = origin_y + (cy + 0.5) * res

        # yaw: fixed (face forward)
        yaw = 0.0
        return wx, wy, yaw

    # -------- Nav2 Action --------
    def _send_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server (navigate_to_pose) not found. Is Nav2 running?")
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        z, w = yaw_to_quat_w(float(yaw))
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        goal.pose = pose

        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        self.last_goal_time = time.time()
        self.goal_sent_time = time.time()
        self.goals_sent += 1

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected.")
            self.active_goal_handle = None
            return

        self.get_logger().info("Goal accepted.")
        self.active_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            res = future.result()
            status = res.status
        except Exception:
            status = -1

        self.get_logger().info(f"Goal finished. status={status}. Choosing a new frontier.")
        self.active_goal_handle = None


def main():
    rclpy.init()
    node = TB3FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
