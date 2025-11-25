#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from team_kalman.msg import PredictedObject


class SafetyNode(Node):

    def __init__(self):
        super().__init__("safety_node")

        # ------------------------------------
        # Subscriptions
        # ------------------------------------
        self.sub_pred = self.create_subscription(
            PredictedObject,
            "/predicted_objects",
            self.pred_callback,
            10
        )

        # ------------------------------------
        # Publishers
        # ------------------------------------
        self.pub_safe_cmd = self.create_publisher(
            Twist,
            "/cmd_vel_safety",
            10
        )

        # ------------------------------------
        # Safety parameters
        # ------------------------------------
        # These radii are *relative to the robot*
        self.stop_radius = 0.5         # guaranteed emergency stop distance
        self.slow_radius = 1.0         # slow-down radius
        self.conf_thresh = 0.4         # confidence threshold

        # Prediction horizon (should match Kalman)
        self.tau = 1.5

        self.get_logger().info("Safety Node (using Kalman predictions) started.")

    # ============================================================
    #   CALLBACK: Receive PredictedObject from the Kalman module
    # ============================================================
    def pred_callback(self, msg: PredictedObject):

        # ------------------------------------
        # 1) Reject low-confidence predictions
        # ------------------------------------
        if msg.confidence < self.conf_thresh:
            return

        # Current filtered robot-frame coordinates
        px = msg.x
        py = msg.y
        distance_now = math.sqrt(px**2 + py**2)

        # Future predicted coordinates (tau seconds in future)
        fx = msg.x_pred
        fy = msg.y_pred
        distance_future = math.sqrt(fx**2 + fy**2)

        # Expand safety radius with predicted object radius
        safety_radius = msg.safety_radius

        twist = Twist()

        # ------------------------------------
        # Check future danger first (more important)
        # ------------------------------------
        if distance_future < safety_radius:
            # HARD STOP
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.get_logger().warn(
                f"EMERGENCY STOP (Predicted)! "
                f"Future dist={distance_future:.2f} < safety={safety_radius:.2f}"
            )

        # ------------------------------------
        # Slow down if predicted object is moderately close
        # ------------------------------------
        elif distance_future < self.slow_radius + safety_radius:
            twist.linear.x = 0.05
            twist.angular.z = 0.0

            self.get_logger().info(
                f"Slow-Down: predicted dist={distance_future:.2f}"
            )

        # ------------------------------------
        # No threat → tell DWA “no override”
        # ------------------------------------
        else:
            twist.linear.x = 999.0
            twist.angular.z = 999.0

        # ------------------------------------
        # Publish override command
        # ------------------------------------
        self.pub_safe_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
