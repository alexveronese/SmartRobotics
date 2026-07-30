"""Detect orange triangular and square prisms in the Gazebo camera image."""

import json
import math
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class ShapeDetector(Node):
    """Segment equal-colour objects and classify them from contour vertices."""

    def __init__(self):
        super().__init__("shape_detector")
        self.declare_parameter("camera_x", 0.52)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 1.35)
        self.declare_parameter("object_plane_z", 0.041)
        self.declare_parameter("horizontal_fov", math.pi / 3.0)
        self.declare_parameter("minimum_area_px", 250.0)

        self._bridge = CvBridge()
        self._camera_info = None
        self._last_log_time = 0.0

        self._detections_pub = self.create_publisher(
            String, "/shape_detections", 10
        )
        self._annotated_pub = self.create_publisher(
            Image, "/shape_detector/annotated", 5
        )
        self._status_pub = self.create_publisher(
            String, "/square_sorting/status", 10
        )
        self.create_subscription(
            CameraInfo,
            "/sorting_camera/camera_info",
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/sorting_camera/image",
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Shape detector ready: classification uses contour vertices, not colour."
        )

    def _camera_info_callback(self, message):
        self._camera_info = message

    def _intrinsics(self, width, height):
        if self._camera_info is not None and self._camera_info.k[0] > 0.0:
            return (
                self._camera_info.k[0],
                self._camera_info.k[4],
                self._camera_info.k[2],
                self._camera_info.k[5],
            )

        horizontal_fov = float(self.get_parameter("horizontal_fov").value)
        focal = width / (2.0 * math.tan(horizontal_fov / 2.0))
        return focal, focal, width / 2.0, height / 2.0

    def _pixel_to_world(self, u, v, width, height):
        fx, fy, cx, cy = self._intrinsics(width, height)
        camera_x = float(self.get_parameter("camera_x").value)
        camera_y = float(self.get_parameter("camera_y").value)
        camera_z = float(self.get_parameter("camera_z").value)
        plane_z = float(self.get_parameter("object_plane_z").value)
        depth = camera_z - plane_z

        # Gazebo cameras look along local +X. The camera is pitched +90 degrees:
        # image-right maps to world -Y and image-down maps to world -X.
        world_x = camera_x - ((v - cy) * depth / fy)
        world_y = camera_y - ((u - cx) * depth / fx)
        return world_x, world_y

    @staticmethod
    def _classify(contour):
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
        vertices = len(polygon)
        if vertices == 3:
            return "triangle", polygon
        if vertices == 4 and cv2.isContourConvex(polygon):
            return "square", polygon
        return "unknown", polygon

    def _image_callback(self, message):
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # cv_bridge reports encoding errors at runtime.
            self.get_logger().error(f"Cannot decode camera image: {error}")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(
            hsv,
            np.array([0, 105, 85], dtype=np.uint8),
            np.array([38, 255, 255], dtype=np.uint8),
        )
        red_wrap = cv2.inRange(
            hsv,
            np.array([170, 105, 85], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        mask = cv2.bitwise_or(orange, red_wrap)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        minimum_area = float(self.get_parameter("minimum_area_px").value)
        detections = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum_area:
                continue
            shape, polygon = self._classify(contour)
            if shape == "unknown":
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0.0:
                continue
            u = moments["m10"] / moments["m00"]
            v = moments["m01"] / moments["m00"]
            world_x, world_y = self._pixel_to_world(
                u, v, image.shape[1], image.shape[0]
            )

            detections.append(
                {
                    "shape": shape,
                    "vertices": int(len(polygon)),
                    "pixel": [round(u, 2), round(v, 2)],
                    "position": [round(world_x, 4), round(world_y, 4)],
                    "area_px": round(area, 1),
                    "on_source_table": bool(world_y > 0.07),
                }
            )

            colour = (55, 220, 55) if shape == "square" else (255, 190, 20)
            cv2.drawContours(image, [polygon], -1, colour, 3)
            cv2.circle(image, (round(u), round(v)), 4, colour, -1)
            cv2.putText(
                image,
                f"{shape} ({world_x:.2f}, {world_y:.2f})",
                (max(0, round(u) - 90), max(20, round(v) - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                colour,
                2,
                cv2.LINE_AA,
            )

        detections.sort(key=lambda item: (item["shape"], item["position"]))
        payload = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "count": len(detections),
            "detections": detections,
        }
        self._detections_pub.publish(String(data=json.dumps(payload)))

        annotated = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        annotated.header = message.header
        self._annotated_pub.publish(annotated)

        now = time.monotonic()
        if detections and now - self._last_log_time > 2.0:
            summary = ", ".join(
                f"{item['shape']}@{tuple(item['position'])}"
                for item in detections
            )
            self.get_logger().info(f"Detected {len(detections)} objects: {summary}")
            self._status_pub.publish(
                String(data=f"PERCEPTION: {len(detections)} objects; {summary}")
            )
            self._last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
