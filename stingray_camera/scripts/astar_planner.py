#!/usr/bin/env python3
import rospy
import numpy as np
import heapq
import math
from scipy.ndimage import binary_dilation
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanResponse


class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        self.robot_radius  = rospy.get_param('~robot_radius', 0.22)
        self.obs_threshold = rospy.get_param('~obstacle_threshold', 50)

        self.map_info      = None
        self.inflated_grid = None   # inflated static map used by A*

        self.path_pub = rospy.Publisher('~planned_path', Path, queue_size=1, latch=True)

        rospy.loginfo("AStarPlanner: waiting for /map ...")
        first_map = rospy.wait_for_message('/map', OccupancyGrid)
        self._map_cb(first_map)

        rospy.Subscriber('/map', OccupancyGrid, self._map_cb)
        rospy.Service('~plan_path', GetPlan, self._plan_srv)
        rospy.loginfo("AStarPlanner: ready")

        rospy.spin()

    # ------------------------------------------------------------------
    @staticmethod
    def _make_disk(r):
        """Circular structuring element of radius r cells."""
        size = 2 * r + 1
        Y, X = np.ogrid[:size, :size]
        return (Y - r)**2 + (X - r)**2 <= r * r

    def _inflate(self, binary, resolution):
        """
        Inflate obstacles by robot_radius using scipy binary_dilation.
        ~100x faster than the old pure Python triple loop.
        """
        r = int(math.ceil(self.robot_radius / resolution))
        return binary_dilation(binary, structure=self._make_disk(r))

    def _map_cb(self, msg):
        """Build inflated grid from map."""
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        raw    = np.array(msg.data, dtype=np.int8).reshape((h, w))
        binary = (raw > self.obs_threshold) | (raw < 0)
        self.inflated_grid = self._inflate(binary, msg.info.resolution)
        rospy.loginfo_once("AStarPlanner: map received and inflated.")

    # ------------------------------------------------------------------
    def world_to_grid(self, wx, wy):
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        return int((wx - ox) / res), int((wy - oy) / res)

    def grid_to_world(self, gx, gy):
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        return gx * res + ox + res * 0.5, gy * res + oy + res * 0.5

    # ------------------------------------------------------------------
    def astar(self, start_w, goal_w):
        if self.inflated_grid is None:
            rospy.logwarn("AStarPlanner: no map yet")
            return None

        start = self.world_to_grid(*start_w)
        goal  = self.world_to_grid(*goal_w)
        h, w  = self.inflated_grid.shape

        def free(x, y):
            return 0 <= x < w and 0 <= y < h and not self.inflated_grid[y, x]

        if not free(*start):
            rospy.logwarn(f"AStarPlanner: start {start} is inside obstacle — nudging")
            nudge_r = 10
            found   = False
            for radius in range(1, nudge_r + 1):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy)) != radius:
                            continue
                        nx, ny = start[0]+dx, start[1]+dy
                        if free(nx, ny):
                            start = (nx, ny); found = True; break
                    if found:
                        break
                if found:
                    break
            if not found:
                rospy.logerr("AStarPlanner: cannot escape start obstacle")
                return None

        if not free(*goal):
            rospy.logwarn(f"AStarPlanner: goal {goal} is inside obstacle")
            return None

        NEIGHBORS = [
            ( 1,  0, 1.0),    (-1,  0, 1.0),
            ( 0,  1, 1.0),    ( 0, -1, 1.0),
            ( 1,  1, 1.414),  (-1,  1, 1.414),
            ( 1, -1, 1.414),  (-1, -1, 1.414),
        ]

        def heuristic(x, y):
            return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)

        open_heap = [(heuristic(*start), 0.0, start)]
        g_score   = {start: 0.0}
        came_from = {}

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)

            if cur == goal:
                path = []
                node = cur
                while node in came_from:
                    path.append(self.grid_to_world(*node))
                    node = came_from[node]
                path.append(self.grid_to_world(*start))
                path.reverse()
                return path

            if g > g_score.get(cur, float('inf')):
                continue

            for dx, dy, cost in NEIGHBORS:
                nb = (cur[0]+dx, cur[1]+dy)
                if not free(*nb):
                    continue
                ng = g_score[cur] + cost
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    heapq.heappush(open_heap, (ng + heuristic(*nb), ng, nb))
                    came_from[nb] = cur

        rospy.logwarn("AStarPlanner: no path found")
        return None

    # ------------------------------------------------------------------
    def _plan_srv(self, req):
        resp    = GetPlanResponse()
        start_w = (req.start.pose.position.x, req.start.pose.position.y)
        goal_w  = (req.goal.pose.position.x,  req.goal.pose.position.y)

        coords = self.astar(start_w, goal_w)
        if coords is None:
            return resp

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = rospy.Time.now()
        for wx, wy in coords:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        resp.plan = path
        self.path_pub.publish(path)
        rospy.loginfo(f"AStarPlanner: path found with {len(coords)} waypoints")
        return resp


if __name__ == '__main__':
    AStarPlanner()
