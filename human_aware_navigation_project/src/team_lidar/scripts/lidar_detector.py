#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from team_lidar.msg import MovingObject


# --------------------------
#  Detection Data Structure
# --------------------------
class Detection:
    def __init__(self):
        # Position in 2D Cartesian coordinates (robot frame: base_link)
        self.x = 0.0
        self.y = 0.0

        # Polar coordinates
        self.r = 0.0
        self.theta = 0.0

        # Estimated motion
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0          # filtered speed used by rest of system
        self.raw_speed = 0.0      # raw speed before thresholding (for confidence)
        self.direction = 0.0

        # Cluster structure
        self.radius = 0.0
        self.num_points = 0

        # Tracking age
        self.age = 0

        # Confidence score
        self.confidence = 0.0


# --------------------------
#  Track Class
# --------------------------
class Track:
    def __init__(self, det: Detection, track_id: int):
        self.id = track_id         # unique ID for this track
        self.det = det
        self.matched = False
        self.last_seen = 0.0



# --------------------------
#  MAIN ROS2 LIDAR NODE
# --------------------------
class LidarDetectorNode(Node):

    def __init__(self):
        super().__init__("lidar_detector")

        # Input: LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # Output: moving objects (custom message)
        self.pub = self.create_publisher(MovingObject, "/moving_objects", 10)

        # LiDAR params
        self.fov = 0.0
        self.angle_increment = 0.0
        self.max_range = 0.0

        self.prev_time = None
        self.points = []
        self.clusters = []
        self.tracks = []
        self.next_track_id = 1

        self.get_logger().info("Lidar Detector Node started with MovingObject msg.")

    # ------------------------------------------------------
    #  LaserScan callback
    # ------------------------------------------------------
    def scan_callback(self, msg: LaserScan):

        if self.prev_time is None:
            self.prev_time = msg.header.stamp
            return

        dt = self.time_diff(self.prev_time, msg.header.stamp)
        if dt <= 0:
            dt = 0.1  # fallback

        self.prev_time = msg.header.stamp

        self.fov = msg.angle_max - msg.angle_min
        self.angle_increment = msg.angle_increment
        self.max_range = msg.range_max

        self.points = self.read_scan(msg)
        self.clusters = self.cluster_points()
        tracks = self.update_tracks(dt)

        for track in tracks:
            det = track.det
            
            msg_out = MovingObject()
            
            msg_out.id = int(track.id)     # <-- NEW
                 
            msg_out.x = float(det.x)
            msg_out.y = float(det.y)
            msg_out.r = float(det.r)
            msg_out.theta = float(det.theta)
                   
            msg_out.vx = float(det.vx)
            msg_out.vy = float(det.vy)
            msg_out.speed = float(det.speed)
            msg_out.direction = float(det.direction)

            msg_out.radius = float(det.radius)
            msg_out.num_points = int(det.num_points)

            msg_out.age = int(det.age)
            msg_out.confidence = float(det.confidence)

            self.pub.publish(msg_out)


    # ------------------------------------------------------
    #  Convert LaserScan → Cartesian
    # ------------------------------------------------------
    def read_scan(self, msg: LaserScan):
        pts = []

        for i, r in enumerate(msg.ranges):

            # filter invalid ranges
            if math.isnan(r) or r <= 0.05 or r >= self.max_range:
                continue

            angle = msg.angle_min + i * self.angle_increment

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # (x, y, range, angle, index)
            pts.append((x, y, r, angle, i))

        return pts

    # ------------------------------------------------------
    #  Clustering
    # ------------------------------------------------------
    def cluster_points(self):
        if not self.points:
            return []

        pts = sorted(self.points, key=lambda p: p[4])

        clusters = []
        current = [pts[0]]

        MAX_GAP = 0.30  # meters

        for i in range(1, len(pts)):
            px, py, _, _, _ = pts[i - 1]
            cx, cy, _, _, _ = pts[i]

            if math.dist((px, py), (cx, cy)) > MAX_GAP:
                if len(current) >= 2:
                    clusters.append(current)
                current = []

            current.append(pts[i])

        if len(current) >= 2:
            clusters.append(current)

        return clusters

    # ------------------------------------------------------
    #  Tracking + Velocity + Confidence
    # ------------------------------------------------------
    def update_tracks(self, dt: float):

        for t in self.tracks:
            t.matched = False

        new_tracks = []
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # smaller association distance to reduce track switching
        MAX_ASSOC = 0.4  # meters

        for cluster in self.clusters:
            det = self.compute_detection(cluster)

            best_i = -1
            best_dist = float("inf")

            for i, t in enumerate(self.tracks):
                if t.matched:
                    continue

                d = math.dist((det.x, det.y), (t.det.x, t.det.y))

                if d < best_dist:
                    best_dist = d
                    best_i = i

            if best_i >= 0 and best_dist < MAX_ASSOC and dt > 0.0:

                prev = self.tracks[best_i]

                dx = det.x - prev.det.x
                dy = det.y - prev.det.y

                # raw speed before thresholding
                raw_vx = dx / dt
                raw_vy = dy / dt
                raw_speed = math.sqrt(raw_vx**2 + raw_vy**2)

                det.vx = raw_vx
                det.vy = raw_vy
                det.speed = raw_speed
                det.raw_speed = raw_speed
                det.direction = math.atan2(det.vy, det.vx)
                det.age = prev.det.age + 1

                # Low-pass smoothing for position
                ALPHA = 0.2
                det.x = ALPHA * det.x + (1 - ALPHA) * prev.det.x
                det.y = ALPHA * det.y + (1 - ALPHA) * prev.det.y

                # Optional: treat tiny speeds as static for velocity fields
                SPEED_EPS = 0.05
                if raw_speed < SPEED_EPS:
                    det.vx = 0.0
                    det.vy = 0.0
                    det.speed = 0.0
                    det.direction = 0.0

                prev.det = det
                prev.last_seen = now_sec
                prev.matched = True

                new_tracks.append(prev)

            else:
                # new track
                det.age = 1
                det.raw_speed = 0.0  # new objects assumed static at birth
                det.confidence = self.compute_confidence(det)

                t = Track(det,self.next_track_id)
                self.next_track_id += 1

                t.last_seen = now_sec
                t.matched = True
                new_tracks.append(t)

        # Remove tracks older than 1 sec
        self.tracks = [t for t in new_tracks if now_sec - t.last_seen < 1.0]

        # Compute confidence for all tracks using raw_speed
        for t in self.tracks:
            t.det.confidence = self.compute_confidence(t.det)

        # Return the active track objects (with IDs)
        return self.tracks

    # ------------------------------------------------------
    #  Compute centroid → Detection
    # ------------------------------------------------------
    def compute_detection(self, cluster):
        d = Detection()

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        d.x = sum(xs) / len(xs)
        d.y = sum(ys) / len(ys)

        d.r = math.sqrt(d.x**2 + d.y**2)
        d.theta = math.atan2(d.y, d.x)

        d.num_points = len(cluster)
        d.radius = max([math.dist((x, y), (d.x, d.y)) for x, y, _, _, _ in cluster])

        return d

    # ------------------------------------------------------
    #  Confidence Score
    # ------------------------------------------------------
    def compute_confidence(self, det: Detection):

        c_points = min(det.num_points / 20.0, 1.0)
        c_age = min(det.age / 10.0, 1.0)
        # use raw_speed so slow-moving humans still counted as "moving"
        c_speed = 1.0 if det.raw_speed > 0.05 else 0.0

        return float(0.5 * c_points + 0.3 * c_age + 0.2 * c_speed)

    # ------------------------------------------------------
    #  Time difference helper
    # ------------------------------------------------------
    def time_diff(self, t1, t2):
        return (t2.sec - t1.sec) + (t2.nanosec - t1.nanosec) * 1e-9


# --------------------------
# MAIN
# --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
