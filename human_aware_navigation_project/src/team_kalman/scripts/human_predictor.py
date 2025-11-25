#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from team_lidar.msg import MovingObject
from team_kalman.msg import PredictedObject


class KalmanPredictorNode(Node):
    """
    Multi-object constant-velocity Kalman predictor.

    - Maintains one KF per MovingObject.id
    - At each update, selects the 'most relevant' object
      (in front of the robot and within a certain range)
    - Publishes a single PredictedObject for that chosen target.

    State per object: x = [x, y, vx, vy]^T  (base_link frame)
    Measurement: z = [x, y]
    """

    def __init__(self):
        super().__init__("kalman_predictor")

        # Subscribe to LiDAR moving-object detections (one message per object)
        self.sub_objects = self.create_subscription(
            MovingObject,
            "/moving_objects",
            self.object_callback,
            20
        )

        # Publish predictions for the most relevant object
        self.pub_pred = self.create_publisher(
            PredictedObject,
            "/predicted_objects",
            10
        )

        # Kalman filter / model parameters
        self.t_horizon = 1.5          # seconds into the future
        self.process_noise = 0.5      # Q scale
        self.measurement_noise = 0.1  # R scale
        self.conf_thresh = 0.4        # ignore very low confidence detections

        # 'Relevance' / risk parameters
        self.front_angle = math.radians(60.0)  # +- 60 degrees considered "in front"
        self.max_considered_range = 5.0        # ignore objects further than this
        self.delete_timeout = 1.0              # seconds without update -> remove track

        # Per-object filters:
        # id -> dict with keys: x (4x1), P (4x4), last_time (float), last_conf (float)
        self.filters = {}

        self.get_logger().info("Kalman Predictor Node (multi-object) started.")

    # ======================================================
    #  Callback: MovingObject messages from LiDAR node
    # ======================================================
    def object_callback(self, obj: MovingObject):
        # Ignore low-confidence detections early
        if obj.confidence < self.conf_thresh:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        obj_id = int(obj.id)

        # --------------------------------------------------
        #  Get or create filter for this ID
        # --------------------------------------------------
        if obj_id not in self.filters:
            # Initialise state with current measurement + LiDAR vx, vy
            x = np.array([
                [obj.x],
                [obj.y],
                [obj.vx],
                [obj.vy]
            ], dtype=float)
            P = np.eye(4) * 0.5  # initial covariance
            self.filters[obj_id] = {
                "x": x,
                "P": P,
                "last_time": now,
                "last_conf": float(obj.confidence)
            }
        else:
            filt = self.filters[obj_id]
            x = filt["x"]
            P = filt["P"]
            last_time = filt["last_time"]

            dt = max(0.01, now - last_time)

            # ---------- PREDICT ----------
            F, Q = self.build_F_Q(dt)
            x = F @ x
            P = F @ P @ F.T + Q

            # ---------- UPDATE ----------
            z = np.array([[obj.x],
                          [obj.y]], dtype=float)
            H = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0]
            ])
            r = self.measurement_noise
            R = np.eye(2) * (r ** 2)

            y = z - (H @ x)                 # innovation
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)  # Kalman gain

            x = x + K @ y
            I = np.eye(4)
            P = (I - K @ H) @ P

            self.filters[obj_id]["x"] = x
            self.filters[obj_id]["P"] = P
            self.filters[obj_id]["last_time"] = now
            self.filters[obj_id]["last_conf"] = float(obj.confidence)

        # --------------------------------------------------
        #  Clean up stale filters (not seen for > delete_timeout)
        # --------------------------------------------------
        to_delete = []
        for fid, f in self.filters.items():
            if now - f["last_time"] > self.delete_timeout:
                to_delete.append(fid)
        for fid in to_delete:
            del self.filters[fid]

        # --------------------------------------------------
        #  Select most relevant object & publish its prediction
        # --------------------------------------------------
        if not self.filters:
            return

        best_id, best_score = self.choose_most_relevant()
        if best_id is None:
            return

        self.publish_prediction(best_id)

    # ======================================================
    #  Helper: Build F and Q (constant-velocity model)
    # ======================================================
    def build_F_Q(self, dt: float):
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        q = self.process_noise
        G = np.array([
            [0.5 * dt ** 2, 0.0],
            [0.0, 0.5 * dt ** 2],
            [dt, 0.0],
            [0.0, dt]
        ])
        Q = (q ** 2) * (G @ G.T)
        return F, Q

    # ======================================================
    #  Choose most relevant (highest risk) object
    # ======================================================
    def choose_most_relevant(self):
        """
        Return (id, score) of the object that is:
        - roughly in front of the robot (within ±front_angle),
        - within max_considered_range,
        - and closest / most dangerous.

        Simple risk score:  (LiDAR_confidence) * (1 / (range + 0.1))
        """
        best_id = None
        best_score = -1.0

        for fid, f in self.filters.items():
            x = f["x"]
            px = float(x[0, 0])
            py = float(x[1, 0])

            r = math.sqrt(px**2 + py**2)
            theta = math.atan2(py, px)

            # Discard if too far or not in front
            if r > self.max_considered_range:
                continue
            if abs(theta) > self.front_angle:
                continue

            lidar_conf = f["last_conf"]
            score = lidar_conf * (1.0 / (r + 0.1))

            if score > best_score:
                best_score = score
                best_id = fid

        return best_id, best_score

    # ======================================================
    #  Publish PredictedObject for a chosen ID
    # ======================================================
    def publish_prediction(self, obj_id: int):
        f = self.filters[obj_id]
        x = f["x"]
        P = f["P"]
        lidar_conf = f["last_conf"]

        # Current filtered state
        px, py, vx, vy = [float(v) for v in x.flatten()]

        # Predict forward by t_horizon
        tau = self.t_horizon
        Fh = np.array([
            [1.0, 0.0, tau, 0.0],
            [0.0, 1.0, 0.0, tau],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        x_future = Fh @ x
        x_pred = float(x_future[0, 0])
        y_pred = float(x_future[1, 0])

        msg_out = PredictedObject()

        # Current filtered state
        msg_out.x = px
        msg_out.y = py
        msg_out.vx = vx
        msg_out.vy = vy

        # Future state at horizon
        msg_out.t_horizon = float(self.t_horizon)
        msg_out.x_pred = x_pred
        msg_out.y_pred = y_pred

        # Safety radius: base + proportional to speed
        speed = math.sqrt(vx**2 + vy**2)
        base_radius = 0.5
        extra = 0.5 * speed
        msg_out.safety_radius = float(base_radius + extra)

        # Confidence: blend LiDAR confidence and filter health
        traceP = float(np.trace(P))
        filt_conf = max(0.0, min(1.0, 3.0 / (3.0 + traceP)))  # heuristic
        msg_out.confidence = float(0.5 * lidar_conf + 0.5 * filt_conf)

        self.pub_pred.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
