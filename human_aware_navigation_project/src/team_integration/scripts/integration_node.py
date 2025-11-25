#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose


class IntegrationNode(Node):

    def __init__(self):
        super().__init__("team_integration")

        # -------------------------------
        # PARAMETERS
        # -------------------------------
        self.declare_parameter("use_rviz_goal", True)
        self.declare_parameter("fixed_goal_x", 2.0)
        self.declare_parameter("fixed_goal_y", 0.0)

        self.use_rviz_goal = self.get_parameter("use_rviz_goal").value

        # Global goal (map frame)
        self.global_goal = None

        if not self.use_rviz_goal:
            gx = float(self.get_parameter("fixed_goal_x").value)
            gy = float(self.get_parameter("fixed_goal_y").value)
            self.global_goal = (gx, gy)
            self.get_logger().info(f"Using FIXED global goal: ({gx}, {gy})")

        # -------------------------------
        # TF setup
        # -------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------------------
        # SUBSCRIBERS
        # -------------------------------
        # From RViz: 2D Nav Goal (in map frame)
        self.sub_goal = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.rviz_goal_callback,
            10
        )

        # DWA velocity
        self.dwa_cmd = None
        self.sub_dwa = self.create_subscription(
            Twist,
            "/cmd_vel_dwa",
            self.dwa_cmd_callback,
            10
        )

        # Safety velocity
        self.safety_cmd = None
        self.sub_safety = self.create_subscription(
            Twist,
            "/cmd_vel_safety",
            self.safety_cmd_callback,
            10
        )

        # -------------------------------
        # PUBLISHERS
        # -------------------------------
        self.pub_local_goal = self.create_publisher(
            PoseStamped,
            "/local_goal",
            10
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("Team Integration Node started.")

    # ----------------------------------------------------------
    # RViz Goal: /goal_pose  (PoseStamped, frame_id=map)
    # ----------------------------------------------------------
    def rviz_goal_callback(self, msg: PoseStamped):
        if self.use_rviz_goal:
            self.global_goal = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(
                f"New RViz global goal set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
            )

    # ----------------------------------------------------------
    # DWA and Safety commands
    # ----------------------------------------------------------
    def dwa_cmd_callback(self, msg: Twist):
        self.dwa_cmd = msg

    def safety_cmd_callback(self, msg: Twist):
        self.safety_cmd = msg

    # ----------------------------------------------------------
    # Publish local goal + final /cmd_vel
    # ----------------------------------------------------------
    def update(self):
        # ------------------------
        # 1) LOCAL GOAL CONVERSION
        # ------------------------
        if self.global_goal is not None:
            local_goal_ps = self.compute_local_goal(self.global_goal)
            if local_goal_ps is not None:
                self.pub_local_goal.publish(local_goal_ps)

        # ------------------------
        # 2) MERGE VELOCITIES
        # ------------------------
        final_cmd = Twist()

        if self.safety_cmd is not None:
            # 999 = "no override"
            if self.safety_cmd.linear.x != 999.0:
                final_cmd = self.safety_cmd
                self.pub_cmd_vel.publish(final_cmd)
                return

        if self.dwa_cmd is not None:
            final_cmd = self.dwa_cmd

        self.pub_cmd_vel.publish(final_cmd)

    # ----------------------------------------------------------
    # Convert global goal (map) → local goal (base_link)
    # ----------------------------------------------------------
    def compute_local_goal(self, goal_xy):
        gx, gy = goal_xy

        # Build PoseStamped in map frame
        map_goal = PoseStamped()
        map_goal.header.frame_id = "map"
        map_goal.pose.position.x = gx
        map_goal.pose.position.y = gy
        map_goal.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException:
            self.get_logger().warning("TF lookup failed (map → base_link)")
            return None

        # Transform map goal → base_link
        local_goal = do_transform_pose(map_goal, transform)
        local_goal.header.frame_id = "base_link"

        return local_goal


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
