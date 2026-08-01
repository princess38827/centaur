#!/usr/bin/env python3
"""
Geofence GPS fixes to a configured property polygon.

Polygon coordinates, including those in YAML, use [longitude, latitude] order.
"""

from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point, Polygon
from std_msgs.msg import Bool, String
import yaml


# Replace these example corners with the actual property boundary.
# Format is (longitude, latitude).
DEFAULT_PROPERTY_POLYGON = [
    (-80.6425, 37.7219),  # NW
    (-80.6416, 37.7226),  # NE
    (-80.6409, 37.7222),  # SE
    (-80.6418, 37.7214),  # SW
]


class GeofenceNode(Node):
    def __init__(self):
        super().__init__("geofence_node")

        self.declare_parameter(
            "polygon_config", "/ws/src/geofence_node/config/property.yaml"
        )
        self.declare_parameter("use_default_polygon", True)
        self.declare_parameter("fail_closed", True)

        config_path = (
            self.get_parameter("polygon_config").get_parameter_value().string_value
        )
        self.property_poly = self.load_polygon(config_path)
        self.get_logger().info(
            f"GEOFENCE LOADED: {len(self.property_poly.exterior.coords)} points"
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/rtk/fix", self.gps_callback, qos
        )
        self.filtered_pub = self.create_publisher(NavSatFix, "/gps/geofenced", qos)
        self.status_pub = self.create_publisher(Bool, "/geofence/inside", qos)
        self.debug_pub = self.create_publisher(String, "/geofence/debug", qos)

        self.last_status = None
        self.outside_count = 0
        self.get_logger().info("Geofence Node READY - Failsafe CLOSED")

    def load_polygon(self, path: str) -> Polygon:
        """Load `polygon: [[lon, lat], ...]` from YAML or use the default."""
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as config_file:
                    data = yaml.safe_load(config_file) or {}
                coords = data.get("polygon", DEFAULT_PROPERTY_POLYGON)
                self.validate_polygon_coordinates(coords)
                polygon = Polygon(coords)
                if not polygon.is_valid or polygon.area == 0:
                    raise ValueError("polygon must be valid and have non-zero area")
                self.get_logger().info(f"Loaded polygon from {path}")
                return polygon
            except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
                self.get_logger().error(f"Failed to load {path}: {error}; using default")

        self.get_logger().warning(
            "Using DEFAULT_PROPERTY_POLYGON - replace it with the property boundary"
        )
        return Polygon(DEFAULT_PROPERTY_POLYGON)

    @staticmethod
    def validate_polygon_coordinates(coords) -> None:
        """Ensure YAML coordinates are [longitude, latitude] pairs."""
        if not isinstance(coords, list) or len(coords) < 3:
            raise ValueError("polygon must contain at least three [lon, lat] pairs")

        for coordinate in coords:
            if not isinstance(coordinate, (list, tuple)) or len(coordinate) != 2:
                raise ValueError("each polygon coordinate must be [lon, lat]")
            longitude, latitude = coordinate
            if not isinstance(longitude, (int, float)) or not isinstance(
                latitude, (int, float)
            ):
                raise ValueError("polygon coordinates must be numeric")
            if not -180 <= longitude <= 180 or not -90 <= latitude <= 90:
                raise ValueError(
                    "polygon coordinates must be [lon, lat] within valid ranges"
                )

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            self.handle_invalid("NO_FIX")
            return

        if not (-90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180):
            self.handle_invalid("INVALID_COORDS")
            return

        point = Point(msg.longitude, msg.latitude)
        inside = self.property_poly.contains(point) or self.property_poly.touches(point)

        self.status_pub.publish(Bool(data=inside))
        if inside:
            self.outside_count = 0
            self.filtered_pub.publish(msg)
            if self.last_status is False:
                self.get_logger().info(
                    f"RE-ENTERED GEOFENCE at {msg.latitude:.6f}, {msg.longitude:.6f}"
                )
            self.last_status = True
            self.debug_pub.publish(
                String(
                    data=(
                        f"INSIDE {datetime.now().isoformat()} "
                        f"{msg.latitude:.6f},{msg.longitude:.6f}"
                    )
                )
            )
        else:
            self.outside_count += 1
            if self.outside_count == 1 or self.outside_count % 50 == 0:
                self.get_logger().warning(
                    f"GEOFENCE VIOLATION REJECTED ({self.outside_count}): "
                    f"{msg.latitude:.6f}, {msg.longitude:.6f} - NOT PUBLISHED"
                )
            self.last_status = False

    def handle_invalid(self, reason: str):
        self.get_logger().error(f"INVALID GPS ({reason}) - BLOCKING")
        self.status_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
