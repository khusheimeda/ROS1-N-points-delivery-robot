#!/usr/bin/env python3
"""
LiDAR-based dynamic obstacle monitor.

KEY BEHAVIOUR:
  - Only stops the robot for NEW obstacles not present in the static map.
  - Mapped obstacles (walls, furniture from GMapping) are ignored entirely.
  - When a new obstacle clears, robot automatically resumes the same path.

Subscribes:
  /scan                   — sensor_msgs/LaserScan
  /amcl_pose              — current robot pose
  /map                    — static occupancy grid (used to filter known obstacles)
  /omni_controller/path   — current planned path to watch

Publishes:
  ~obstacle_detected    — Bool(True) when NEW obstacle blocks path, False when clear
  /cmd_vel              — zero Twist for instant stop (bypasses controller latency)
"""
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool


class ObstacleMonitor:
    def __init__(self):
        rospy.init_node('obstacle_monitor')

        # FIX 1: Increased danger_dist and lookahead so robot detects earlier
        self.danger_dist  = rospy.get_param('~danger_distance', 1.0)
        self.path_width   = rospy.get_param('~path_width',      0.35)
        self.lookahead    = rospy.get_param('~lookahead_wps',   20)
        self.map_thresh   = rospy.get_param('~map_threshold',   50)
        # Must match astar_planner's robot_radius so _is_mapped_obstacle checks
        # the same inflation halo that A* uses when planning paths near walls.
        self.robot_radius = rospy.get_param('~robot_radius',    0.22)

        self.pose     = None
        self.path     = []
        self.blocked  = False

        # FIX 2: static map stored as numpy array for fast cell lookup
        self.map_data = None   # numpy (H x W) int8 array
        self.map_info = None   # OccupancyGrid metadata

        rospy.Subscriber('/amcl_pose',            PoseWithCovarianceStamped, self._pose_cb)
        rospy.Subscriber('/scan',                 LaserScan,                 self._scan_cb)
        rospy.Subscriber('/omni_controller/path', Path,                      self._path_cb)
        rospy.Subscriber('/map',                  OccupancyGrid,             self._map_cb)

        self.obs_pub = rospy.Publisher('~obstacle_detected', Bool,  queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel',           Twist, queue_size=1)

        rospy.loginfo("ObstacleMonitor: ready")
        rospy.loginfo(f"  danger_distance:   {self.danger_dist}m")
        rospy.loginfo(f"  lookahead_wps:     {self.lookahead}")
        rospy.loginfo(f"  Mapped obstacles:  IGNORED (only new obstacles trigger stop)")
        rospy.spin()

    # ------------------------------------------------------------------ callbacks
    def _pose_cb(self, msg):
        p = msg.pose.pose
        siny = 2.0*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y)
        cosy = 1.0 - 2.0*(p.orientation.y**2 + p.orientation.z**2)
        self.pose = (p.position.x, p.position.y, math.atan2(siny, cosy))

    def _map_cb(self, msg):
        """Cache static map as numpy array for fast per-cell lookup."""
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
                            (msg.info.height, msg.info.width))
        rospy.loginfo_once("ObstacleMonitor: static map received and cached")

    def _path_cb(self, msg):
        self.path    = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.blocked = False
        rospy.loginfo(f"ObstacleMonitor: new path received ({len(self.path)} waypoints)")

    # ------------------------------------------------------------------ map lookup
    def _is_mapped_obstacle(self, wx, wy):
        """
        Return True if (wx, wy) is within A*'s inflation halo of any
        occupied cell in the static map.

        We use robot_radius (same value as astar_planner) to compute
        check_r. This is the key insight: A* plans paths along the EDGE
        of the inflation halo. So a LiDAR hit on the sofa edge will land
        up to robot_radius away from the actual occupied sofa cell.
        If we only check 1-2 cells we miss it. Checking robot_radius cells
        matches exactly what A* considers "part of the obstacle".
        """
        if self.map_data is None or self.map_info is None:
            return False

        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        H   = self.map_info.height
        W   = self.map_info.width

        cgx = int((wx - ox) / res)
        cgy = int((wy - oy) / res)

        # check_r matches A*'s inflation radius in cells
        # e.g. robot_radius=0.22m, res=0.05m → check_r = ceil(0.22/0.05) = 5 cells
        check_r = int(math.ceil(self.robot_radius / res))

        for dy in range(-check_r, check_r + 1):
            for dx in range(-check_r, check_r + 1):
                gx = cgx + dx
                gy = cgy + dy
                if not (0 <= gx < W and 0 <= gy < H):
                    continue
                val = int(self.map_data[gy, gx])
                # Only OCCUPIED cells (not unknown=-1, not free=0)
                if val > self.map_thresh:
                    return True

        return False

    # ------------------------------------------------------------------ scan
    def _scan_cb(self, msg):
        if self.pose is None or not self.path:
            return

        rx, ry, rth = self.pose

        # Compute the direction the robot is heading along the path
        # Use the next waypoint to determine forward direction
        ahead = self.path[:self.lookahead]
        if not ahead:
            return

        # Direction to the next waypoint (this is the "forward" direction)
        next_wx, next_wy = ahead[0]
        path_angle = math.atan2(next_wy - ry, next_wx - rx)

        # Convert LiDAR hits to world coords
        # Only consider hits within the path corridor — ignore everything else
        new_obstacles = []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment + rth
            ox = rx + r * math.cos(angle)
            oy = ry + r * math.sin(angle)

            # Only consider obstacles within danger distance
            if math.hypot(ox - rx, oy - ry) >= self.danger_dist:
                continue

            # Check if this hit is within path_width of the planned path
            # If not near the path corridor at all, skip immediately
            # This replaces the 30° cone with the actual path shape
            near_path = False
            prev = (rx, ry)
            for (wx, wy) in ahead:
                if self._pt_seg_dist(ox, oy, prev[0], prev[1], wx, wy) < self.path_width:
                    near_path = True
                    rospy.logdebug(f"  LiDAR index {i}: r={r:.2f}m angle={math.degrees(angle - rth):.1f}° "
                                   f"world=({ox:.2f},{oy:.2f}) → within path corridor")
                    break
                prev = (wx, wy)

            if not near_path:
                continue   # not near the path — ignore entirely

            # Skip walls/furniture already in the static map
            if self._is_mapped_obstacle(ox, oy):
                continue

            new_obstacles.append((ox, oy))

        # If any new obstacles remain, path is blocked
        path_blocked = len(new_obstacles) > 0

        if path_blocked and not self.blocked:
            rospy.logwarn("ObstacleMonitor: 🛑 NEW obstacle on path — stopping")
            rospy.logwarn(f"  Blocking obstacles ({len(new_obstacles)} total):")
            for (ox, oy) in new_obstacles:
                dist = math.hypot(ox - rx, oy - ry)
                angle_to_obs = math.degrees(math.atan2(oy - ry, ox - rx) - rth)
                rospy.logwarn(f"    → world=({ox:.2f},{oy:.2f})  dist={dist:.2f}m  "
                              f"angle={angle_to_obs:.1f}° from robot heading")
            self.vel_pub.publish(Twist())        # instant stop
            self.blocked = True
            self.obs_pub.publish(Bool(data=True))

        elif not path_blocked and self.blocked:
            rospy.loginfo("ObstacleMonitor: ✅ path clear — resuming")
            self.blocked = False
            self.obs_pub.publish(Bool(data=False))

        elif path_blocked and self.blocked:
            self.vel_pub.publish(Twist())        # keep overriding controller

    # ------------------------------------------------------------------ geometry
    @staticmethod
    def _pt_seg_dist(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        len2   = dx*dx + dy*dy
        if len2 < 1e-9:
            return math.sqrt((px-ax)**2 + (py-ay)**2)
        t   = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / len2))
        return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)


if __name__ == '__main__':
    ObstacleMonitor()
