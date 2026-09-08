#!/usr/bin/env python3
"""
Holonomic pure-pursuit path follower for the omnidirectional Triton robot.

Unlike RTR (rotate-translate-rotate), this controller exploits the robot's
ability to move in any direction without first rotating. It picks a
lookahead point along the path and commands a body-frame (vx, vy) velocity
toward it. Heading is controlled independently (held at initial yaw, or
optionally aligned with the path tangent).

Subscribes:
  /amcl_pose            — current robot pose from AMCL
  ~path                 — nav_msgs/Path to follow

Publishes:
  /cmd_vel              — geometry_msgs/Twist (linear.x, linear.y, angular.z)
  ~status               — String: 'idle' | 'tracking' | 'arrived'
  ~arrived              — Bool(True) when final waypoint reached
"""
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String


class OmniController:
    def __init__(self):
        rospy.init_node('omni_controller')

        self.lookahead   = rospy.get_param('~lookahead_dist',    0.40)   # m
        self.max_lin     = rospy.get_param('~max_linear',        0.20)   # m/s
        self.max_ang     = rospy.get_param('~max_angular',       1.00)   # rad/s
        self.kp_lin      = rospy.get_param('~kp_linear',         1.0)
        self.kp_ang      = rospy.get_param('~kp_angular',        1.5)
        self.goal_tol    = rospy.get_param('~goal_tolerance',    0.15)   # m
        self.slow_radius = rospy.get_param('~slow_radius',       0.40)   # m: ramp down within this of goal
        # heading_mode: 'hold' = lock initial yaw, 'tangent' = face along path
        self.heading_mode = rospy.get_param('~heading_mode', 'hold')

        self.pose         = None     # (x, y, yaw)
        self.path         = []       # list of (wx, wy)
        self.idx          = 0        # nearest-point cursor (monotonic)
        self.active       = False
        self.paused       = False    # True = obstacle present, hold position
        self.locked_yaw   = None     # yaw to hold when heading_mode == 'hold'

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._pose_cb)
        rospy.Subscriber('~path',      Path,                       self._path_cb)
        rospy.Subscriber('/obstacle_monitor/obstacle_detected', Bool, self._obstacle_cb)

        self.cmd_pub     = rospy.Publisher('/cmd_vel',  Twist,  queue_size=1)
        self.status_pub  = rospy.Publisher('~status',   String, queue_size=1)
        self.arrived_pub = rospy.Publisher('~arrived',  Bool,   queue_size=1)

        rospy.loginfo("OmniController: ready  heading_mode=%s  lookahead=%.2fm",
                      self.heading_mode, self.lookahead)
        rospy.Timer(rospy.Duration(0.05), self._control_loop)
        rospy.spin()

    # ------------------------------------------------------------------
    def _pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, self._quat_yaw(p.orientation))

    def _obstacle_cb(self, msg):
        if msg.data and not self.paused:
            # Obstacle appeared — pause but keep path in memory
            self.paused = True
            self._stop()
            self.status_pub.publish(String(data='paused'))
            rospy.logwarn("OmniController: ⏸ paused — obstacle on path")
        elif not msg.data and self.paused:
            # Obstacle cleared — resume following same path
            self.paused = False
            self.status_pub.publish(String(data='tracking'))
            rospy.loginfo("OmniController: ▶ resuming — path clear")

    def _path_cb(self, msg):
        if not msg.poses:
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.idx = 0
        self.active = True
        self.locked_yaw = self.pose[2] if self.pose is not None else 0.0
        rospy.loginfo("OmniController: new path, %d points", len(self.path))

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ------------------------------------------------------------------
    def _advance_cursor(self, rx, ry):
        """Move idx forward to the closest path point at or after current idx."""
        best_i = self.idx
        best_d = float('inf')
        # Only scan forward from current cursor — prevents going backwards.
        for i in range(self.idx, len(self.path)):
            wx, wy = self.path[i]
            d = (wx - rx)**2 + (wy - ry)**2
            if d < best_d:
                best_d = d
                best_i = i
        self.idx = best_i

    def _lookahead_point(self, rx, ry):
        """Return path point at least lookahead away, walking forward from idx."""
        L2 = self.lookahead * self.lookahead
        for i in range(self.idx, len(self.path)):
            wx, wy = self.path[i]
            if (wx - rx)**2 + (wy - ry)**2 >= L2:
                return (wx, wy), i
        # No point further than lookahead → use final goal
        return self.path[-1], len(self.path) - 1

    # ------------------------------------------------------------------
    def _control_loop(self, _event):
        if not self.active or self.pose is None or not self.path:
            return

        # Paused = obstacle on path, hold position until it clears
        if self.paused:
            self._stop()
            return

        rx, ry, rth = self.pose
        gx, gy = self.path[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)

        # Arrival check
        if dist_to_goal < self.goal_tol:
            self._stop()
            self.active = False
            self.arrived_pub.publish(Bool(data=True))
            self.status_pub.publish(String(data='arrived'))
            rospy.loginfo("OmniController: arrived at goal")
            return

        self._advance_cursor(rx, ry)
        (lx, ly), li = self._lookahead_point(rx, ry)

        # World-frame error vector toward lookahead
        ex = lx - rx
        ey = ly - ry
        norm = math.hypot(ex, ey)
        if norm < 1e-6:
            self._stop()
            return

        # Speed: ramp down as we approach final goal
        speed = self.max_lin
        if dist_to_goal < self.slow_radius:
            speed = max(0.05, self.max_lin * (dist_to_goal / self.slow_radius))

        # Unit vector in world frame, scaled by desired speed
        vx_w = (ex / norm) * speed
        vy_w = (ey / norm) * speed

        # Rotate world-frame velocity into robot body frame
        c, s = math.cos(rth), math.sin(rth)
        vx_b =  c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w

        # Heading control
        if self.heading_mode == 'tangent':
            target_yaw = math.atan2(ey, ex)
        else:  # 'hold'
            target_yaw = self.locked_yaw if self.locked_yaw is not None else rth
        yaw_err = self._wrap(target_yaw - rth)
        wz = float(np.clip(self.kp_ang * yaw_err, -self.max_ang, self.max_ang))

        cmd = Twist()
        cmd.linear.x  = float(np.clip(vx_b, -self.max_lin, self.max_lin))
        cmd.linear.y  = float(np.clip(vy_b, -self.max_lin, self.max_lin))
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data='tracking'))

    def _stop(self):
        self.cmd_pub.publish(Twist())


if __name__ == '__main__':
    OmniController()
