#!/usr/bin/env python3
"""
Multi-goal delivery orchestrator with OBSTACLE DETECTION + LED FEEDBACK.

Sets delivery goals via rosparam ~goals (list of {x, y, name}).
Ordering strategies: fixed | greedy | optimal  (set ~ordering_strategy).

Flow:
  1. Order remaining goals with chosen strategy.
  2. Call A* planner service → send path to OMNI controller.
  3. Wait for arrival OR obstacle-detected signal.
  4. On obstacle: turn LED RED, fetch updated map, replan same goal.
  5. On arrival: mark delivered, pick next goal.
  6. When all goals delivered: turn LED RAINBOW.

LED colors published to /cmd_color (Int32):
  0 = OFF
  1 = RED       (obstacle detected — robot stopped/replanning)
  2 = GREEN     (navigating — robot moving)
  3 = YELLOW    (delivered a goal, brief flash)
  4 = BLUE      (idle / waiting)
  5 = RAINBOW   (all deliveries complete!)
"""
import rospy
import math
import itertools
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool, String, Int32
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import Empty


# LED Color Constants (must match led_controller.py)
LED_OFF      = 0
LED_RED      = 1   # Obstacle detected
LED_GREEN    = 2   # Navigating (static green)
LED_YELLOW   = 3   # Delivered (brief)
LED_BLUE     = 4   # Idle
LED_RAINBOW  = 5   # All done!
LED_SIREN    = 6   # 🚨 Police siren — used while moving between points


class DeliveryManager:
    def __init__(self):
        rospy.init_node('delivery_manager')
        rospy.loginfo("="*70)
        rospy.loginfo("DeliveryManager: Initializing...")
        rospy.loginfo("="*70)

        strategy   = rospy.get_param('~ordering_strategy', 'greedy')
        goals_raw  = rospy.get_param('~goals', [])

        self.strategy = strategy
        self.goals    = [(float(g['x']), float(g['y']), g.get('name', f'G{i}'))
                         for i, g in enumerate(goals_raw)]

        rospy.loginfo(f"Configuration loaded:")
        rospy.loginfo(f"  Strategy: {self.strategy}")
        rospy.loginfo(f"  Number of goals: {len(self.goals)}")
        for i, (x, y, name) in enumerate(self.goals):
            rospy.loginfo(f"    Goal {i+1}: {name} at ({x:.2f}, {y:.2f})")

        self.pose          = None
        self.arrived       = False
        self.obstacle_flag = False
        self.updated_map   = None

        rospy.loginfo("Setting up subscribers...")
        rospy.Subscriber('/amcl_pose',
                         PoseWithCovarianceStamped, self._pose_cb)
        rospy.Subscriber('/omni_controller/arrived',
                         Bool, self._arrived_cb)
        rospy.Subscriber('/obstacle_monitor/obstacle_detected',
                         Bool, self._obstacle_cb)
        rospy.Subscriber('/obstacle_monitor/updated_map',
                         OccupancyGrid, self._map_update_cb)
        rospy.loginfo("  ✓ Subscribed to /amcl_pose")
        rospy.loginfo("  ✓ Subscribed to /omni_controller/arrived")
        rospy.loginfo("  ✓ Subscribed to /obstacle_monitor/obstacle_detected")
        rospy.loginfo("  ✓ Subscribed to /obstacle_monitor/updated_map")

        rospy.loginfo("Setting up publishers...")
        self.path_pub = rospy.Publisher('/omni_controller/path',
                                        Path,  queue_size=1)
        # Publishes symbolic codes (RED/GREEN/RAINBOW) to led_controller,
        # which translates them to 24-bit RGB hex on /cmd_color.
        self.led_pub  = rospy.Publisher('/led_command',
                                        Int32, queue_size=1, latch=True)
        rospy.loginfo("  ✓ Publishing to /omni_controller/path")
        rospy.loginfo("  ✓ Publishing to /led_command")

        # Default LED state
        self._led(LED_BLUE)  # Idle / waiting

        rospy.loginfo("Waiting for /astar_planner/plan_path service...")
        try:
            rospy.wait_for_service('/astar_planner/plan_path', timeout=200.0)
            self._plan_srv = rospy.ServiceProxy('/astar_planner/plan_path', GetPlan)
            rospy.loginfo("  ✓ A* planner service connected")
        except rospy.ROSException:
            rospy.logerr("  ✗ TIMEOUT: A* planner service not available!")
            rospy.logerr("  Make sure astar_planner node is running")
            return

        # Optional: wire up the dynamic-obstacle clear service if available.
        # Called between goals to drop stale blockers — fresh scans will re-add
        # anything that's still really there.
        self._clear_dynamic = None
        try:
            rospy.wait_for_service('/astar_planner/clear_dynamic_obstacles', timeout=2.0)
            self._clear_dynamic = rospy.ServiceProxy(
                '/astar_planner/clear_dynamic_obstacles', Empty)
            rospy.loginfo("  ✓ Dynamic-obstacle clear service connected")
        except rospy.ROSException:
            rospy.logwarn("  (dynamic-obstacle clear service unavailable — "
                          "continuing without between-goal cleanup)")

        rospy.loginfo("="*70)
        rospy.loginfo("DeliveryManager: Ready!")
        rospy.loginfo("="*70)
        rospy.loginfo("Waiting 2 seconds for AMCL to converge...")
        rospy.sleep(2.0)
        
        self._run()

    # ------------------------------------------------------------------  callbacks
    def _pose_cb(self, msg):
        p = msg.pose.pose
        siny = 2.0*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y)
        cosy = 1.0 - 2.0*(p.orientation.y**2 + p.orientation.z**2)
        new_pose = (p.position.x, p.position.y, math.atan2(siny, cosy))
        if self.pose is None:
            rospy.loginfo(f"✓ First AMCL pose received: ({new_pose[0]:.2f}, {new_pose[1]:.2f}, {new_pose[2]:.2f} rad)")
        self.pose = new_pose

    def _arrived_cb(self, msg):
        if msg.data:
            rospy.loginfo("✓ OMNI controller reports: ARRIVED at goal")
            self.arrived = True

    def _obstacle_cb(self, msg):
        if msg.data:
            rospy.logwarn("🛑 Obstacle detected — robot paused")
            self.obstacle_flag = True
            self.arrived       = False
            self._led(LED_RED)   # magenta = stopped
        else:
            rospy.loginfo("✅ Obstacle cleared — robot resuming")
            self.obstacle_flag = False
            self._led(LED_SIREN)  # back to siren = moving

    def _map_update_cb(self, msg):
        rospy.loginfo("Updated map received from obstacle monitor")
        self.updated_map = msg

    # ------------------------------------------------------------------  helpers
    @staticmethod
    def _pose_stamped(x, y):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        return ps

    def _call_planner(self, sx, sy, gx, gy):
        rospy.loginfo(f"  Calling A* planner: ({sx:.2f}, {sy:.2f}) → ({gx:.2f}, {gy:.2f})")
        req       = GetPlanRequest()
        req.start = self._pose_stamped(sx, sy)
        req.goal  = self._pose_stamped(gx, gy)
        req.tolerance = 0.2
        try:
            resp = self._plan_srv(req)
            if resp.plan and resp.plan.poses:
                rospy.loginfo(f"  ✓ A* returned path with {len(resp.plan.poses)} waypoints")
                return resp.plan
            else:
                rospy.logwarn(f"  ✗ A* returned empty path")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"  ✗ A* planner service error: {e}")
            return None

    def _path_cost(self, plan):
        if not plan or not plan.poses:
            return float('inf')
        total = 0.0
        ps = plan.poses
        for i in range(1, len(ps)):
            dx = ps[i].pose.position.x - ps[i-1].pose.position.x
            dy = ps[i].pose.position.y - ps[i-1].pose.position.y
            total += math.sqrt(dx*dx + dy*dy)
        return total

    def _led(self, color):
        """Set LED color and log it"""
        color_names = {
            LED_OFF: 'OFF',
            LED_RED: '🟣 MAGENTA (stopped — obstacle!)',
            LED_GREEN: '🟢 GREEN',
            LED_YELLOW: '🟡 YELLOW (delivered)',
            LED_BLUE: '🔵 BLUE (idle)',
            LED_RAINBOW: '🌈 RAINBOW (goal reached!)',
            LED_SIREN: '🚨 SIREN (en route)'
        }
        rospy.loginfo(f"LED: {color_names.get(color, 'UNKNOWN')}")
        self.led_pub.publish(Int32(data=color))

    # ------------------------------------------------------------------  ordering
    def _order(self, start_xy, remaining):
        rospy.loginfo(f"Computing order for {len(remaining)} goals using '{self.strategy}' strategy")
        
        if self.strategy == 'fixed':
            rospy.loginfo("  Using fixed order (as provided)")
            return list(remaining)

        if self.strategy == 'greedy':
            rospy.loginfo("  Using greedy (nearest neighbor) algorithm")
            ordered   = []
            cur       = start_xy
            unvisited = list(remaining)
            iteration = 0
            
            while unvisited:
                iteration += 1
                rospy.loginfo(f"  Iteration {iteration}: {len(unvisited)} goals remaining")
                costs = []
                for g in unvisited:
                    plan = self._call_planner(cur[0], cur[1], g[0], g[1])
                    cost = self._path_cost(plan) if plan and plan.poses else float('inf')
                    costs.append(cost)
                    rospy.loginfo(f"    Cost to {g[2]}: {cost:.2f}m")
                
                best = int(np.argmin(costs)) if costs else 0
                selected = unvisited.pop(best)
                ordered.append(selected)
                rospy.loginfo(f"  → Selected: {selected[2]} (cost: {costs[best]:.2f}m)")
                cur = (ordered[-1][0], ordered[-1][1])
            
            return ordered

        if self.strategy == 'optimal':
            n = len(remaining)
            if n > 8:
                rospy.logwarn(f"  {n} goals > 8, falling back to greedy (optimal too slow)")
                self.strategy = 'greedy'
                return self._order(start_xy, remaining)
            
            rospy.loginfo(f"  Computing optimal order (trying all {math.factorial(n)} permutations)")
            nodes = [start_xy] + [(g[0], g[1]) for g in remaining]
            C = {}
            
            rospy.loginfo("  Building cost matrix...")
            for i in range(len(nodes)):
                for j in range(len(nodes)):
                    if i != j:
                        plan = self._call_planner(nodes[i][0], nodes[i][1],
                                                  nodes[j][0], nodes[j][1])
                        C[(i,j)] = self._path_cost(plan) if plan and plan.poses else float('inf')
            
            rospy.loginfo("  Evaluating all permutations...")
            best_cost = float('inf')
            best_perm = list(range(n))
            
            for perm in itertools.permutations(range(n)):
                cost = C[(0, perm[0]+1)]
                for k in range(len(perm)-1):
                    cost += C[(perm[k]+1, perm[k+1]+1)]
                if cost < best_cost:
                    best_cost = cost
                    best_perm = list(perm)
            
            rospy.loginfo(f"  ✓ Optimal order found! Total cost: {best_cost:.2f}m")
            return [remaining[i] for i in best_perm]

        rospy.logwarn(f"Unknown strategy '{self.strategy}', using fixed order")
        return list(remaining)

    # ------------------------------------------------------------------  navigation
    def _navigate_to(self, gx, gy, name):
        """Navigate to (gx,gy). Returns True on arrival, False if unreachable."""
        rospy.loginfo("-"*70)
        rospy.loginfo(f"Navigating to: {name} ({gx:.2f}, {gy:.2f})")
        rospy.loginfo("-"*70)

        if self.pose is None:
            rospy.logwarn("Waiting for AMCL pose...")
            return False

        rospy.loginfo(f"Current position: ({self.pose[0]:.2f}, {self.pose[1]:.2f})")
        self._led(LED_SIREN)

        plan = self._call_planner(self.pose[0], self.pose[1], gx, gy)
        if plan is None or not plan.poses:
            rospy.logerr(f"✗ No valid path to {name}! Goal unreachable.")
            self._led(LED_RED)
            return False

        path_length = self._path_cost(plan)
        rospy.loginfo(f"✓ Path planned: {len(plan.poses)} waypoints, {path_length:.2f}m total")

        self.arrived       = False
        self.obstacle_flag = False

        rospy.loginfo(f"Publishing path to OMNI controller...")
        self.path_pub.publish(plan)
        rospy.loginfo(f"✓ Path published. Waiting for robot to navigate...")

        rate = rospy.Rate(10)
        wait_time = 0.0

        while not rospy.is_shutdown():
            if self.arrived:
                rospy.loginfo(f"✓✓✓ SUCCESS! Arrived at {name} after {wait_time:.1f} seconds")
                return True

            wait_time += 0.1
            if wait_time % 5.0 < 0.1:
                if self.obstacle_flag:
                    rospy.loginfo(f"  ⏸ Waiting for obstacle to clear... ({wait_time:.0f}s elapsed)")
                else:
                    rospy.loginfo(f"  Still navigating... ({wait_time:.0f}s elapsed)")

            rate.sleep()

        return False

    # ------------------------------------------------------------------  main loop
    def _run(self):
        rospy.loginfo("="*70)
        rospy.loginfo("STARTING DELIVERY RUN")
        rospy.loginfo("="*70)
        
        # Wait for AMCL pose
        timeout = 200.0
        elapsed = 0.0
        while self.pose is None and not rospy.is_shutdown():
            if elapsed >= timeout:
                rospy.logerr("✗ TIMEOUT: No AMCL pose received!")
                rospy.logerr("  Make sure AMCL is running and initialized")
                return
            rospy.loginfo(f"Waiting for AMCL pose... ({elapsed:.1f}s)")
            rospy.sleep(0.5)
            elapsed += 0.5

        rospy.loginfo(f"✓ Robot localized at ({self.pose[0]:.2f}, {self.pose[1]:.2f})")

        remaining = list(self.goals)
        delivered = 0
        total     = len(remaining)

        rospy.loginfo("="*70)
        rospy.loginfo(f"DELIVERY PLAN: {total} goals, strategy='{self.strategy}'")
        rospy.loginfo("="*70)

        while remaining and not rospy.is_shutdown():
            rospy.loginfo(f"\n{'='*70}")
            rospy.loginfo(f"Progress: {delivered}/{total} delivered, {len(remaining)} remaining")
            rospy.loginfo(f"{'='*70}")

            # Wipe any stale dynamic obstacles from previous goals. The next
            # LiDAR scan will re-add anything that's actually still there.
            if self._clear_dynamic is not None:
                try:
                    self._clear_dynamic()
                except rospy.ServiceException as e:
                    rospy.logwarn(f"  (clear-dynamic-obstacles failed: {e})")

            ordered     = self._order(self.pose[:2], remaining)
            next_goal   = ordered[0]
            gx, gy, name = next_goal

            rospy.loginfo(f"Next goal: {name} at ({gx:.2f}, {gy:.2f})")
            success = self._navigate_to(gx, gy, name)

            remaining.remove(next_goal)
            
            if success:
                delivered += 1
                # 🌈 RAINBOW celebration at every goal arrival!
                self._led(LED_RAINBOW)
                rospy.loginfo(f"✓ {name} delivered successfully! ({delivered}/{total} complete)")
                rospy.loginfo("Celebrating with rainbow for 2 seconds...")
                rospy.sleep(2.0)
            else:
                rospy.logwarn(f"✗ {name} skipped (unreachable)")

        rospy.loginfo("="*70)
        rospy.loginfo(f"DELIVERY COMPLETE: {delivered}/{total} successful")
        if delivered == total:
            rospy.loginfo("🎉🌈 ALL GOALS DELIVERED! 🌈🎉")
            self._led(LED_RAINBOW)   # keep rainbow on for the grand finale
        else:
            rospy.logwarn(f"⚠ {total - delivered} goals were skipped")
            self._led(LED_RED)
        rospy.loginfo("="*70)


if __name__ == '__main__':
    try:
        DeliveryManager()
    except rospy.ROSInterruptException:
        rospy.loginfo("DeliveryManager interrupted by user")
    except Exception as e:
        rospy.logerr(f"DeliveryManager crashed: {e}")
        import traceback
        traceback.print_exc()
