#!/usr/bin/env python3

import os
import json
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_depth = None

        self.camera_info_ready = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # RGB image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Depth image (for real robot / OAK-D)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Camera intrinsic parameters
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Mission trigger from teammate A
        self.nav_sub = self.create_subscription(
            String,
            '/mission/navigation',
            self.navigation_callback,
            10
        )

        # Publish result for logger / teammate C
        self.vision_pub = self.create_publisher(
            String,
            '/mission/vision',
            10
        )

        self.output_dir = '/root/ros2_ws/vision_outputs'
        os.makedirs(self.output_dir, exist_ok=True)

        # Fallback scale estimation if no depth is available
        # You may tune this based on cone size later
        self.real_cone_width_m = 0.28

        self.get_logger().info('Vision node v2 started.')

    # ----------------------------
    # ROS callbacks
    # ----------------------------
    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion failed: {e}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_ready = True

    def navigation_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Invalid navigation JSON: {e}')
            return

        wp_id = data.get('wp_id', -1)
        status = data.get('status', '')

        if status != 'reached':
            return

        if self.latest_image is None:
            self.get_logger().warn('No RGB image available yet.')
            return

        result = self.process_image(self.latest_image.copy(), wp_id)

        out = String()
        out.data = json.dumps(result)
        self.vision_pub.publish(out)

        self.get_logger().info(f'Published vision result: {out.data}')

    # ----------------------------
    # Main pipeline
    # ----------------------------
    def process_image(self, image, wp_id):
        annotated = image.copy()

        # 1) Crop to middle horizontal band
        band, y1, y2 = self.crop_middle_band(image)

        # 2) Detect cone inside band
        cone_det = self.detect_cone(band)
        if cone_det is None:
            filename = f'wp{wp_id}_shape.jpg'
            cv2.putText(
                annotated, 'Cone not found', (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            cv2.imwrite(os.path.join(self.output_dir, filename), annotated)

            return {
                'obj_color': 'unknown',
                'obj_shape': 'unknown',
                'dist': -1.0,
                'img_path': filename
            }

        cone_x, cone_y, cone_w, cone_h, cone_cnt = cone_det
        cone_y_global = cone_y + y1

        # Draw cone bbox on full image
        cv2.rectangle(
            annotated,
            (cone_x, cone_y_global),
            (cone_x + cone_w, cone_y_global + cone_h),
            (0, 140, 255),
            2
        )
        cv2.putText(
            annotated,
            'cone',
            (cone_x, max(20, cone_y_global - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 140, 255),
            2
        )

        # 3) Detect unknown object (yellow / dark green) inside same band
        obj_det = self.detect_unknown_object(band)
        if obj_det is None:
            filename = f'wp{wp_id}_shape.jpg'
            cv2.putText(
                annotated, 'Object not found', (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            cv2.imwrite(os.path.join(self.output_dir, filename), annotated)

            return {
                'obj_color': 'unknown',
                'obj_shape': 'unknown',
                'dist': -1.0,
                'img_path': filename
            }

        obj_color, obj_cnt, (obj_x, obj_y, obj_w, obj_h) = obj_det
        obj_y_global = obj_y + y1

        shape_name = self.classify_shape(obj_cnt)

        # 4) Estimate distance from object to cone
        dist = self.estimate_obj_to_cone_distance(
            (cone_x, cone_y_global, cone_w, cone_h),
            (obj_x, obj_y_global, obj_w, obj_h)
        )

        # 5) Draw object
        cv2.rectangle(
            annotated,
            (obj_x, obj_y_global),
            (obj_x + obj_w, obj_y_global + obj_h),
            (0, 255, 0),
            2
        )
        label = f'{obj_color} {shape_name} {dist:.2f}m'
        cv2.putText(
            annotated,
            label,
            (obj_x, max(20, obj_y_global - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        # Draw ROI band for debugging
        cv2.line(annotated, (0, y1), (annotated.shape[1], y1), (255, 0, 0), 2)
        cv2.line(annotated, (0, y2), (annotated.shape[1], y2), (255, 0, 0), 2)

        filename = f'wp{wp_id}_shape.jpg'
        cv2.imwrite(os.path.join(self.output_dir, filename), annotated)

        return {
            'obj_color': obj_color,
            'obj_shape': shape_name,
            'dist': float(round(dist, 2)),
            'img_path': filename
        }

    # ----------------------------
    # ROI
    # ----------------------------
    def crop_middle_band(self, image):
        h, w = image.shape[:2]
        y1 = int(0.25 * h)
        y2 = int(0.80 * h)
        return image[y1:y2, :], y1, y2

    # ----------------------------
    # Cone detection
    # ----------------------------
    def detect_cone(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # orange cone
        mask = cv2.inRange(hsv, (5, 120, 120), (22, 255, 255))
        mask = cv2.medianBlur(mask, 5)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # cone often taller than wide
            if h < w:
                continue

            if area > best_area:
                best_area = area
                best = (x, y, w, h, cnt)

        return best

    # ----------------------------
    # Unknown object detection
    # ----------------------------
    def detect_unknown_object(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        masks = {
            # yellow plastic box
            'yellow': cv2.inRange(hsv, (18, 90, 90), (38, 255, 255)),
            # dark green bucket / object
            'green': cv2.inRange(hsv, (35, 40, 20), (85, 255, 140)),
        }

        best = None
        best_area = 0
        roi_area = image.shape[0] * image.shape[1]

        for color_name, mask in masks.items():
            mask = cv2.medianBlur(mask, 5)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 400:
                    continue

                if area > 0.25 * roi_area:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)

                # reject very flat grass-like regions
                if w / float(h + 1e-6) > 3.0:
                    continue

                if area > best_area:
                    best_area = area
                    best = (color_name, cnt, (x, y, w, h))

        return best

    # ----------------------------
    # Shape classification
    # ----------------------------
    def classify_shape(self, contour):
        peri = cv2.arcLength(contour, True)
        if peri <= 0:
            return 'unknown'

        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)
        area = cv2.contourArea(contour)

        circularity = 4.0 * math.pi * area / (peri * peri)

        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            ratio = w / float(h + 1e-6)
            if 0.85 <= ratio <= 1.15:
                return 'square'
            return 'rectangle'

        if circularity > 0.75:
            return 'circle'

        return 'unknown'

    # ----------------------------
    # Depth-based distance
    # ----------------------------
    def get_depth_median(self, depth_img, u, v, window=5):
        h, w = depth_img.shape[:2]
        u = int(u)
        v = int(v)

        x1 = max(0, u - window)
        x2 = min(w, u + window + 1)
        y1 = max(0, v - window)
        y2 = min(h, v + window + 1)

        patch = depth_img[y1:y2, x1:x2].astype(np.float32)

        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0]

        if valid.size == 0:
            return None

        depth = np.median(valid)

        # if value looks like millimeters, convert to meters
        if depth > 20.0:
            depth = depth / 1000.0

        return float(depth)

    def pixel_to_3d(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z], dtype=np.float32)

    def estimate_obj_to_cone_distance_depth(self, cone_bbox, obj_bbox):
        if self.latest_depth is None or not self.camera_info_ready:
            return None

        cone_x, cone_y, cone_w, cone_h = cone_bbox
        obj_x, obj_y, obj_w, obj_h = obj_bbox

        cone_u = cone_x + cone_w / 2.0
        cone_v = cone_y + cone_h / 2.0
        obj_u = obj_x + obj_w / 2.0
        obj_v = obj_y + obj_h / 2.0

        cone_z = self.get_depth_median(self.latest_depth, cone_u, cone_v)
        obj_z = self.get_depth_median(self.latest_depth, obj_u, obj_v)

        if cone_z is None or obj_z is None:
            return None

        p_cone = self.pixel_to_3d(cone_u, cone_v, cone_z)
        p_obj = self.pixel_to_3d(obj_u, obj_v, obj_z)

        return float(np.linalg.norm(p_obj - p_cone))

    # ----------------------------
    # Fallback distance (no depth)
    # ----------------------------
    def estimate_obj_to_cone_distance_fallback(self, cone_bbox, obj_bbox):
        cone_x, cone_y, cone_w, cone_h = cone_bbox
        obj_x, obj_y, obj_w, obj_h = obj_bbox

        cone_cx = cone_x + cone_w / 2.0
        cone_cy = cone_y + cone_h / 2.0
        obj_cx = obj_x + obj_w / 2.0
        obj_cy = obj_y + obj_h / 2.0

        pixel_dist = math.sqrt((obj_cx - cone_cx) ** 2 + (obj_cy - cone_cy) ** 2)

        if cone_w <= 0:
            return -1.0

        meters_per_pixel = self.real_cone_width_m / float(cone_w)
        return float(pixel_dist * meters_per_pixel)

    def estimate_obj_to_cone_distance(self, cone_bbox, obj_bbox):
        dist = self.estimate_obj_to_cone_distance_depth(cone_bbox, obj_bbox)
        if dist is not None:
            return dist

        return self.estimate_obj_to_cone_distance_fallback(cone_bbox, obj_bbox)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()