#!/usr/bin/env python3
"""
RTR (Rotate–Translate–Rotate) path-following controller.

Subscribes:
  /amcl_pose            — current robot pose from AMCL
  ~path                 — nav_msgs/Path to follow (published by delivery_manager)

Publishes:
  /cmd_vel              — velocity commands
  ~status               — String: 'idle' | 'rotating' | 'translating' | 'arrived'
  ~arrived              — Bool(True) when final waypoint reached
"""
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String


class RTRController:
    def __init__(self):
        rospy.init_node('rtr_controller')

        self.kp_ang      = rospy.get_param('~kp_angular',        1.5)
        self.kp_lin      = rospy.get_param('~kp_linear',         0.4)
        self.max_lin     = rospy.get_param('~max_linear',        0.20)
        self.max_ang     = rospy.get_param('~max_angular',       1.00)
        self.goal_tol    = rospy.get_param('~goal_tolerance',    0.15)   # metres
        self.head_tol    = rospy.get_param('~heading_tolerance', 0.10)   # radians
        self.skip        = rospy.get_param('~waypoint_skip',     3)

        self.pose    = None   # (x, y, yaw)
        self.path    = []     # list of (wx, wy)
        self.idx     = 0
        self.active  = False

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._pose_cb)
        rospy.Subscriber('~path',      Path,                       self._path_cb)

        self.cmd_pub     = rospy.Publisher('/cmd_vel',   Twist,  queue_size=1)
        self.status_pub  = rospy.Publisher('~status',   String, queue_size=1)
        self.arrived_pub = rospy.Publisher('~arrived',  Bool,   queue_size=1)

        rospy.loginfo("RTRController: ready")
        rospy.Timer(rospy.Duration(0.1), self._control_loop)
        rospy.spin()

    # ------------------------------------------------------------------
    def _pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, self._quat_yaw(p.orientation))

    def _path_cb(self, msg):
        poses = msg.poses
        if not poses:
            return
        # Downsample: every skip-th point + always include the last
        wps = [(p.pose.position.x, p.pose.position.y) for p in poses[::self.skip]]
        last = (poses[-1].pose.position.x, poses[-1].pose.position.y)
        if not wps or wps[-1] != last:
            wps.append(last)
        self.path   = wps
        self.idx    = 0
        self.active = True
        rospy.loginfo(f"RTRController: new path, {len(self.path)} waypoints")

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
    def _control_loop(self, _event):
        if not self.active or self.pose is None:
            return

        if self.idx >= len(self.path):
            self._stop()
            self.active = False
            self.arrived_pub.publish(Bool(data=True))
            self.status_pub.publish(String(data='arrived'))
            rospy.loginfo("RTRController: arrived at goal")
            return

        wx, wy = self.path[self.idx]
        rx, ry, rth = self.pose
        dx, dy = wx - rx, wy - ry
        dist   = math.sqrt(dx*dx + dy*dy)
        target = math.atan2(dy, dx)
        herr   = self._wrap(target - rth)

        cmd = Twist()

        if abs(herr) > self.head_tol:
            # Phase 1 – rotate to face waypoint
            cmd.angular.z = float(np.clip(self.kp_ang * herr, -self.max_ang, self.max_ang))
            self.status_pub.publish(String(data='rotating'))
        elif dist > self.goal_tol:
            # Phase 2 – translate (with soft heading correction)
            cmd.linear.x  = float(np.clip(self.kp_lin * dist,  0.0, self.max_lin))
            cmd.angular.z = float(np.clip(self.kp_ang * herr,
                                          -self.max_ang * 0.3, self.max_ang * 0.3))
            self.status_pub.publish(String(data='translating'))
        else:
            # Waypoint reached — advance
            self.idx += 1

        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())


if __name__ == '__main__':
    RTRController()

