#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from led_ring import detectionBlink, initStrip, matchSonar, startupBlink
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        # Node configuration
        self.declare_parameter(
            "front_angle_range", 35.0
        )  # Check ±35° in front: [-35°; 35°] scan range
        self.declare_parameter("min_distance", 0.1)  # Minimum valid distance (m)
        self.declare_parameter("max_distance", 1.0)  # Maximum detection range (m)
        self.declare_parameter(
            "cluster_tolerance", 0.1
        )  # Max gap between points in same object (m)
        self.declare_parameter(
            "expected_object_width", 0.1
        )  # Expected object width (m)
        self.declare_parameter("width_tolerance", 0.05)  # ±tolerance for width matching
        self.declare_parameter(
            "normalization", False
        )  # Set to True to get a [0°; 2*front_angle°] scan range
        self.declare_parameter("led_ring", False)  # To enable led_ring output
        self.declare_parameter("debug", False)

        self.front_angle = self.get_parameter("front_angle_range").value
        self.min_dist = self.get_parameter("min_distance").value
        self.max_dist = self.get_parameter("max_distance").value
        self.cluster_tol = self.get_parameter("cluster_tolerance").value
        self.expected_width = self.get_parameter("expected_object_width").value
        self.width_tol = self.get_parameter("width_tolerance").value
        self.normalization = self.get_parameter("normalization").value
        self.led_ring = self.get_parameter("led_ring").value
        self.debug = self.get_parameter("debug").value

        # Subscribe to lidar scan from ldlidar_stl_ros2 package
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        # Create a publisher for visualization with rviz2
        self.marker_pub = self.create_publisher(MarkerArray, "/detected_objects", 10)

        # Create a publisher for UART sender
        self.uart_data_pub = self.create_publisher(Int16MultiArray, "/uart_data", 10)

        self.get_logger().info("object Detector started!")
        self.get_logger().info(
            f"Looking for object {self.expected_width}m wide in front in a [{-self.front_angle + (self.front_angle if self.normalization else 0)};{self.front_angle + (self.front_angle if self.normalization else 0)}]° range"
        )

        if self.led_ring:
            initStrip()
            startupBlink()

    def scan_callback(self, msg: LaserScan):
        """Core of the program. Publish the marker_array of detected object and the data for the uart interface

        Args:
            msg: incoming data from the lidar
        """

        if self.led_ring:
            matchSonar(True)

        # Get frame_id from laser scan for proper visualization
        self.frame_id = msg.header.frame_id

        if self.debug:
            # Print detailed info only for first scan
            if not hasattr(self, "_structure_printed"):
                self.get_logger().info("=== LaserScan Structure ===")
                self.get_logger().info(f"Frame: {msg.header.frame_id}")
                self.get_logger().info(f"Points: {len(msg.ranges)}")
                self.get_logger().info(
                    f"Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°"
                )
                self.get_logger().info(
                    f"Angle step: {math.degrees(msg.angle_increment):.3f}°"
                )
                self.get_logger().info(
                    f"Distance range: {msg.range_min}m to {msg.range_max}m"
                )
                self.get_logger().info(f"Sample ranges: {msg.ranges[:10]}")
                self.get_logger().info(f"Has intensities: {len(msg.intensities) > 0}")
                self._structure_printed = True

        # Extract front sector data
        front_points = self.get_front_sector(msg)

        # If nothing is detected just leave
        if len(front_points) == 0:
            self.get_logger().info("Empty front sector data")
            return

        # Create marker_array for visualization
        marker_array = MarkerArray()
        marker_id = 0

        # Add detection_zone_marker for visualization
        detection_zone_marker = self.create_detection_zone_marker()
        marker_array.markers.append(detection_zone_marker)

        # Still show zone even with no detections
        if len(front_points) == 0:
            self.marker_pub.publish(marker_array)
            return

        # Find clusters (potential objects)
        clusters = self.find_clusters(front_points)

        # Check if there is a object in each cluster
        for cluster in clusters:
            distance, width, angle = self.analyze_cluster(cluster)

            # Check if it matches our expected object based on expected_width and width_tol
            is_object = abs(width - self.expected_width) <= self.width_tol
            if is_object:
                if self.debug:
                    self.get_logger().info(
                        f"Object DETECTED! Distance: {distance:.2f}m, "
                        f"Width: {width:.2f}m, Angle: {angle + (self.front_angle if self.normalization else 0):.1f}°"
                    )

                if self.led_ring:
                    matchSonar(False)
                    detectionBlink()

                # If an object is detected we send the distance and angle of the cluster to the UART sender
                msg = Int16MultiArray()

                # Distance is formatted to be an int that represent millimeter
                # Angle is formatted to be an int that represent a degree in [-angle + (self.front_angle if self.normalization else 0); angle + (self.front_angle if self.normalization else 0)] range
                msg.data = [
                    int(distance * 1000),
                    int(angle + (self.front_angle if self.normalization else 0)),
                ]
                self.uart_data_pub.publish(msg)

            # Create marker for this cluster
            marker = self.create_cluster_marker(cluster, marker_id, is_object)
            marker_array.markers.append(marker)
            marker_id += 1

        # Publish markers
        self.marker_pub.publish(marker_array)

        matchSonar(True)

    def create_cluster_marker(self, cluster, marker_id, is_object):
        """Create a visual marker for a cluster"""

        cluster_marker = Marker()
        cluster_marker.header.frame_id = self.frame_id
        cluster_marker.header.stamp = self.get_clock().now().to_msg()
        cluster_marker.ns = "clusters"
        cluster_marker.id = marker_id
        cluster_marker.type = Marker.LINE_STRIP
        cluster_marker.action = Marker.ADD

        # Draw lines connecting all points in cluster
        for point in cluster:
            p = Point()
            p.x = point["x"]
            p.y = point["y"]
            p.z = 0.0
            cluster_marker.points.append(p)

        # Close the shape
        p = Point()
        p.x = cluster[0]["x"]
        p.y = cluster[0]["y"]
        p.z = 0.0
        cluster_marker.points.append(p)

        # Color: green if target object, red otherwise
        cluster_marker.scale.x = 0.02  # Line width
        if is_object:
            cluster_marker.color.r = 0.0
            cluster_marker.color.g = 1.0  # Green
            cluster_marker.color.b = 0.0
        else:
            cluster_marker.color.r = 1.0  # Red
            cluster_marker.color.g = 0.0
            cluster_marker.color.b = 0.0
        cluster_marker.color.a = 1.0

        return cluster_marker

    def create_detection_zone_marker(self):
        """Create a visual marker showing the front detection zone"""

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detection_zone"
        marker.id = 9999  # Unique ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Create arc showing detection zone
        # Start at object position
        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0
        marker.points.append(p)

        # Draw left boundary
        left_angle = math.radians(self.front_angle)
        p = Point()
        p.x = self.max_dist * math.cos(left_angle)
        p.y = self.max_dist * math.sin(left_angle)
        p.z = 0.0
        marker.points.append(p)

        # Draw arc at max distance
        num_arc_points = 20
        for i in range(num_arc_points + 1):
            angle = math.radians(
                self.front_angle - (2 * self.front_angle * i / num_arc_points)
            )
            p = Point()
            p.x = self.max_dist * math.cos(angle)
            p.y = self.max_dist * math.sin(angle)
            p.z = 0.0
            marker.points.append(p)

        # Draw right boundary back to object
        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0
        marker.points.append(p)

        # Style: Semi-transparent blue
        marker.scale.x = 0.03  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0  # Blue
        marker.color.a = 0.5  # Semi-transparent

        return marker

    def get_front_sector(self, msg):
        """Extract points in front of object (±front_angle degrees)"""

        points = []

        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if distance < self.min_dist or distance > self.max_dist:
                continue

            # Calculate angle for this point
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            # Normalize angle to -180 to 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            # Check if in front sector
            if abs(angle_deg) <= self.front_angle:
                # Convert to Cartesian coordinates
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append(
                    {"x": x, "y": y, "distance": distance, "angle": angle_deg}
                )

        return points

    def find_clusters(self, points):
        """Group nearby points into clusters, a cluster is very likely an object

        Args:
            points ([TODO:parameter]): [TODO:description]

        Returns:
            clusters: List of detected cluster. Cluster are a list of points.
        """

        if len(points) == 0:
            return []

        # Sort by angle for easier clustering
        points = sorted(points, key=lambda p: p["angle"])

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev_point = points[i - 1]
            curr_point = points[i]

            # Calculate distance between consecutive points
            dx = curr_point["x"] - prev_point["x"]
            dy = curr_point["y"] - prev_point["y"]
            gap = math.sqrt(dx * dx + dy * dy)

            if gap <= self.cluster_tol:
                # Same object
                current_cluster.append(curr_point)
            else:
                # New object - save previous cluster
                if len(current_cluster) >= 3:  # Need at least 3 points for valid object
                    clusters.append(current_cluster)
                current_cluster = [curr_point]

        # Don't forget last cluster
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)

        return clusters

    def analyze_cluster(self, cluster):
        """Calculate distance, width and angle of a cluster

        Args:
            cluster ([TODO:parameter]): List point that are closed to each other

        Returns:
            avg_distance: Average distance of the cluster
            width: Width of the cluster
            avg_angle: Average angle of the cluster
        """

        # Average distance to object
        avg_distance = sum(p["distance"] for p in cluster) / len(cluster)

        # Calculate width (distance between leftmost and rightmost points)
        leftmost = min(cluster, key=lambda p: p["y"])
        rightmost = max(cluster, key=lambda p: p["y"])

        dx = rightmost["x"] - leftmost["x"]
        dy = rightmost["y"] - leftmost["y"]
        width = math.sqrt(dx * dx + dy * dy)

        # Average angle
        avg_angle = sum(p["angle"] for p in cluster) / len(cluster)

        return avg_distance, width, avg_angle


def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
