#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def clamp(v: float, v_min: float, v_max: float) -> float:
    return max(v_min, min(v_max, v))


class AdaptiveDWANode(Node):
    """
    Simplified Dynamic Window Approach (DWA) local planner with
    a lightweight adaptive behaviour:
      - slows down max translational velocity when obstacles are close.

    Subscribes:
      - /odom        (nav_msgs/Odometry)
      - /scan        (sensor_msgs/LaserScan)
      - /local_goal  (geometry_msgs/PoseStamped)  # goal relative to robot

    Publishes:
      - /cmd_vel_dwa (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('adaptive_dwa')

        # ---------------- Parameters ----------------
        self.declare_parameter('max_v', 0.4)
        self.declare_parameter('min_v', 0.0)
        self.declare_parameter('max_omega', 1.0)
        self.declare_parameter('max_acc_v', 0.4)
        self.declare_parameter('max_acc_omega', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 1.0)
        self.declare_parameter('robot_radius', 0.25)

        # Adaptive behaviour wrt obstacle distance
        self.declare_parameter('safe_obstacle_dist', 0.6)
        self.declare_parameter('danger_obstacle_dist', 0.3)

        # Cost weights
        self.declare_parameter('heading_weight', 1.0)
        self.declare_parameter('clearance_weight', 0.8)
        self.declare_parameter('velocity_weight', 0.2)

        # Load params
        self.max_v = float(self.get_parameter('max_v').value)
        self.min_v = float(self.get_parameter('min_v').value)
        self.max_omega = float(self.get_parameter('max_omega').value)
        self.max_acc_v = float(self.get_parameter('max_acc_v').value)
        self.max_acc_omega = float(self.get_parameter('max_acc_omega').value)
        self.dt = float(self.get_parameter('dt').value)
        self.predict_time = float(self.get_parameter('predict_time').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)

        self.safe_obstacle_dist = float(self.get_parameter('safe_obstacle_dist').value)
        self.danger_obstacle_dist = float(self.get_parameter('danger_obstacle_dist').value)

        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.velocity_weight = float(self.get_parameter('velocity_weight').value)

        # ---------------- State ----------------
        self.current_v = 0.0
        self.current_omega = 0.0
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None
        self.goal: Optional[PoseStamped] = None

        # ---------------- Subs & Pub ----------------
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.sub_goal = self.create_subscription(
            PoseStamped, '/local_goal', self.goal_callback, 10
        )

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_dwa', 10)

        # Main loop
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Adaptive DWA Node started.')

    # ---------------- Callbacks ----------------
    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.current_v = msg.twist.twist.linear.x
        self.current_omega = msg.twist.twist.angular.z

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg

    # ---------------- Core DWA loop ----------------
    def update(self):
        if self.odom is None or self.scan is None or self.goal is None:
            return

        # Dynamic window for v, w
        dw_v_min = max(self.min_v, self.current_v - self.max_acc_v * self.dt)
        dw_v_max = min(self.max_v, self.current_v + self.max_acc_v * self.dt)

        dw_w_min = -self.max_omega
        dw_w_max = self.max_omega

        # Adaptive: shrink max velocity based on closest obstacle
        min_scan_dist = self.get_min_scan_distance()
        adaptive_max_v = self.compute_adaptive_max_v(min_scan_dist)
        dw_v_max = min(dw_v_max, adaptive_max_v)

        best_score = -1e9
        best_v = 0.0
        best_w = 0.0

        # Sample velocities
        v_samples = 5
        w_samples = 11
        for i in range(v_samples):
            v = dw_v_min + (dw_v_max - dw_v_min) * i / max(1, v_samples - 1)
            for j in range(w_samples):
                w = dw_w_min + (dw_w_max - dw_w_min) * j / max(1, w_samples - 1)

                traj = self.rollout_trajectory(v, w)
                heading_cost = self.compute_heading_cost(traj)
                clearance_cost = self.compute_clearance_cost(traj)
                velocity_cost = v / self.max_v if self.max_v > 0.0 else 0.0

                score = (self.heading_weight * heading_cost +
                         self.clearance_weight * clearance_cost +
                         self.velocity_weight * velocity_cost)

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w

        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.pub_cmd.publish(cmd)

    # ---------------- Helpers ----------------
    def get_min_scan_distance(self) -> float:
        if self.scan is None:
            return float('inf')
        dists = [r for r in self.scan.ranges if not math.isnan(r) and r > 0.0]
        if not dists:
            return float('inf')
        return min(dists)

    def compute_adaptive_max_v(self, min_dist: float) -> float:
        """
        Shrink max_v when obstacles are close.
        - If min_dist <= danger_obstacle_dist: stop.
        - If min_dist >= safe_obstacle_dist: full max_v.
        - Between: linearly scale.
        """
        if min_dist <= self.danger_obstacle_dist:
            return 0.0
        if min_dist >= self.safe_obstacle_dist:
            return self.max_v

        alpha = (min_dist - self.danger_obstacle_dist) / (
            self.safe_obstacle_dist - self.danger_obstacle_dist
        )
        alpha = clamp(alpha, 0.0, 1.0)
        return alpha * self.max_v

    def rollout_trajectory(self, v: float, w: float) -> List[Tuple[float, float]]:
        """
        Simulate forward for predict_time with control (v, w),
        returning a list of (x, y) positions in base_link frame.
        Start at (0,0,0) – local relative motion only.
        """
        x = 0.0
        y = 0.0
        theta = 0.0
        traj = []
        t = 0.0
        while t < self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            traj.append((x, y))
            t += self.dt
        return traj

    def compute_heading_cost(self, traj: List[Tuple[float, float]]) -> float:
        """
        Heading cost: how well the endpoint points toward the goal.
        Treat goal as a point in front of robot.
        """
        if self.goal is None or not traj:
            return 0.0
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        goal_angle = math.atan2(gy, gx)

        end_x, end_y = traj[-1]
        end_angle = math.atan2(end_y, end_x)

        angle_diff = abs(self.wrap_angle(goal_angle - end_angle))
        # Normalise to [0,1], 1 = perfect alignment
        return 1.0 - angle_diff / math.pi

    def compute_clearance_cost(self, traj: List[Tuple[float, float]]) -> float:
        """
        Clearance cost: minimum distance from trajectory points to obstacles.
        Normalised so that > robot_radius is good and <=0 is collision.
        """
        if self.scan is None or not traj:
            return 0.0

        obstacles = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if math.isnan(r) or r <= 0.0 or r > self.scan.range_max:
                angle += self.scan.angle_increment
                continue
            ox = r * math.cos(angle)
            oy = r * math.sin(angle)
            obstacles.append((ox, oy))
            angle += self.scan.angle_increment

        if not obstacles:
            return 1.0  # no obstacles -> best clearance

        min_dist = float('inf')
        for x, y in traj:
            for ox, oy in obstacles:
                d = math.hypot(ox - x, oy - y)
                if d < min_dist:
                    min_dist = d

        if min_dist <= self.robot_radius:
            return 0.0  # collision
        max_clear = 2.0  # cap for normalisation
        return clamp((min_dist - self.robot_radius) / (max_clear - self.robot_radius), 0.0, 1.0)

    @staticmethod
    def wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveDWANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
