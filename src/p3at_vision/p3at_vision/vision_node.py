#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class DepthFrame:
    stamp_sec: float
    image: np.ndarray


@dataclass
class FrameSample:
    stamp_sec: float
    rgb: np.ndarray
    depth: Optional[np.ndarray]


@dataclass
class Detection:
    color: str
    contour: np.ndarray
    bbox: Tuple[int, int, int, int]
    score: float
    class_name: str = "unknown"
    shape_name: str = "unknown"


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self.declare_parameter("run_id", "")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("navigation_topic", "/mission/navigation")
        self.declare_parameter("vision_topic", "/mission/vision")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("save_dir", str(Path.cwd() / "artifacts" / "part2_runs" / "vision_dev" / "photos"))
        self.declare_parameter("save_raw", True)
        self.declare_parameter("save_annotated", True)
        self.declare_parameter("frame_buffer_size", 12)
        self.declare_parameter("max_depth_age_sec", 0.25)
        self.declare_parameter("min_post_inspect_frames", 2)
        self.declare_parameter("inspect_timeout_sec", 1.5)

        self.run_id = str(self.get_parameter("run_id").value).strip() or "vision_dev"
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.navigation_topic = str(self.get_parameter("navigation_topic").value)
        self.vision_topic = str(self.get_parameter("vision_topic").value)
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.save_raw = bool(self.get_parameter("save_raw").value)
        self.save_annotated = bool(self.get_parameter("save_annotated").value)
        self.max_depth_age_sec = float(self.get_parameter("max_depth_age_sec").value)
        self.min_post_inspect_frames = max(1, int(self.get_parameter("min_post_inspect_frames").value))
        self.inspect_timeout_sec = max(0.1, float(self.get_parameter("inspect_timeout_sec").value))
        buffer_size = max(4, int(self.get_parameter("frame_buffer_size").value))

        self.save_root = Path(str(self.get_parameter("save_dir").value)).expanduser()
        if not self.save_root.is_absolute():
            self.save_root = Path.cwd() / self.save_root
        self.marker_dir = self.save_root / "marker"
        self.object_dir = self.save_root / "object"
        self.annotated_dir = self.save_root / "annotated"
        self.marker_dir.mkdir(parents=True, exist_ok=True)
        self.object_dir.mkdir(parents=True, exist_ok=True)
        self.annotated_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frames: Deque[FrameSample] = deque(maxlen=buffer_size)
        self.depth_frames: Deque[DepthFrame] = deque(maxlen=max(8, buffer_size * 3))
        self.last_processed_key: Optional[Tuple[str, int]] = None
        self.pending_nav_event: Optional[Dict] = None
        self.pending_request_key: Optional[Tuple[str, int]] = None
        self.pending_request_started_sec: Optional[float] = None

        self.camera_info_ready = False
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.image_sub = self.create_subscription(Image, self.rgb_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.nav_sub = self.create_subscription(String, self.navigation_topic, self.navigation_callback, 10)
        self.vision_pub = self.create_publisher(String, self.vision_topic, 10)
        self.pending_timer = self.create_timer(0.1, self.pending_request_timeout_cb)

        self.get_logger().info(
            f"Vision node ready. rgb={self.rgb_topic} depth={self.depth_topic} cam_info={self.camera_info_topic}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"RGB conversion failed: {exc}")
            return

        stamp_sec = stamp_to_sec(msg.header.stamp)
        depth = self.match_depth_frame(stamp_sec)
        self.frames.append(FrameSample(stamp_sec=stamp_sec, rgb=rgb, depth=depth))
        self.try_process_pending_request()

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Depth conversion failed: {exc}")
            return
        depth_frame = DepthFrame(stamp_sec=stamp_to_sec(msg.header.stamp), image=depth)
        self.depth_frames.append(depth_frame)
        self.backfill_rgb_depth(depth_frame)
        self.try_process_pending_request()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_info_ready = self.fx > 0.0 and self.fy > 0.0

    def navigation_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Invalid navigation JSON: {exc}")
            return

        if data.get("status") != "reached" or data.get("phase") != "inspect":
            return

        wp_id = str(data.get("wp_id", ""))
        attempt_idx = int(data.get("attempt_idx", 0))
        key = (wp_id, attempt_idx)
        if key == self.last_processed_key or key == self.pending_request_key:
            return

        nav_ts = float(data.get("ts", 0.0))
        self.pending_nav_event = data
        self.pending_request_key = key
        self.pending_request_started_sec = self.now_sec()
        if nav_ts > 0.0:
            self.frames = deque(
                (frame for frame in self.frames if frame.stamp_sec >= nav_ts),
                maxlen=self.frames.maxlen,
            )
        self.try_process_pending_request()

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def match_depth_frame(self, stamp_sec: float) -> Optional[np.ndarray]:
        if not self.use_depth or not self.depth_frames:
            return None
        best = min(self.depth_frames, key=lambda frame: abs(frame.stamp_sec - stamp_sec))
        if abs(best.stamp_sec - stamp_sec) > self.max_depth_age_sec:
            return None
        return best.image.copy()

    def backfill_rgb_depth(self, depth_frame: DepthFrame) -> None:
        if not self.use_depth:
            return
        for frame in self.frames:
            if frame.depth is not None:
                continue
            if abs(frame.stamp_sec - depth_frame.stamp_sec) <= self.max_depth_age_sec:
                frame.depth = depth_frame.image.copy()

    def pending_request_timeout_cb(self) -> None:
        if self.pending_nav_event is None or self.pending_request_started_sec is None:
            return
        if (self.now_sec() - self.pending_request_started_sec) < self.inspect_timeout_sec:
            return
        self.try_process_pending_request(force=True)

    def try_process_pending_request(self, force: bool = False) -> None:
        if self.pending_nav_event is None or self.pending_request_key is None:
            return

        result = self.process_burst(self.pending_nav_event, force=force)
        if result is None:
            return

        wp_id = str(self.pending_nav_event.get("wp_id", ""))
        attempt_idx = int(self.pending_nav_event.get("attempt_idx", 0))
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self.vision_pub.publish(out)
        self.last_processed_key = self.pending_request_key
        self.pending_nav_event = None
        self.pending_request_key = None
        self.pending_request_started_sec = None
        self.get_logger().info(f"Published vision result for wp={wp_id} attempt={attempt_idx}: {result['status']}")

    def process_burst(self, nav_event: Dict, force: bool = False) -> Optional[Dict]:
        wp_id = str(nav_event.get("wp_id", ""))
        attempt_idx = int(nav_event.get("attempt_idx", 0))
        result = self._base_result(wp_id, attempt_idx)

        if not self.camera_info_ready:
            result["status"] = "failed"
            result["error_reason"] = "camera_info_unavailable"
            return result

        if not self.frames:
            if force:
                result["status"] = "failed"
                result["error_reason"] = "rgb_timeout"
                return result
            return None

        nav_ts = float(nav_event.get("ts", 0.0))
        candidate_frames = list(self.frames)
        if nav_ts > 0.0:
            candidate_frames = [frame for frame in candidate_frames if frame.stamp_sec >= nav_ts]
        if not candidate_frames:
            if force:
                result["status"] = "failed"
                result["error_reason"] = "post_inspect_rgb_timeout"
                return result
            return None
        if len(candidate_frames) < self.min_post_inspect_frames and not force:
            return None

        best_result = None
        best_score = -1.0
        for frame in candidate_frames:
            candidate = self.process_frame(frame, wp_id, attempt_idx)
            score = float(candidate.pop("_score", -1.0))
            if score > best_score:
                best_score = score
                best_result = candidate

        if best_result is None:
            result["status"] = "failed"
            result["error_reason"] = "no_usable_frame"
            return result
        return best_result

    def process_frame(self, frame: FrameSample, wp_id: str, attempt_idx: int) -> Dict:
        image = frame.rgb.copy()
        annotated = image.copy()
        result = self._base_result(wp_id, attempt_idx)

        cone_det = self.detect_cone(image)
        if cone_det is None:
            result["status"] = "failed"
            result["error_reason"] = "marker_not_found"
            result["_score"] = 0.0
            return result

        obj_det = self.detect_object(image, cone_det.bbox)
        if obj_det is None:
            marker_img_path = self.save_crop(image, cone_det.bbox, self.marker_dir, f"wp{wp_id}_attempt{attempt_idx}_marker.jpg")
            annotated_img_path = self.save_annotated_image(
                annotated,
                cone_det,
                None,
                wp_id,
                attempt_idx,
                status_text="object_not_found",
            )
            result.update(
                {
                    "status": "partial",
                    "error_reason": "object_not_found",
                    "marker_img_path": marker_img_path,
                    "annotated_img_path": annotated_img_path,
                }
            )
            marker_point = self.point_from_detection(frame.depth, cone_det)
            if marker_point is not None:
                result["marker_pose_camera"] = self.point_to_dict(marker_point)
            result["_score"] = cone_det.score
            return result

        self.draw_detection(annotated, cone_det, label="marker")
        self.draw_detection(annotated, obj_det, label=f"{obj_det.color} {obj_det.shape_name}")

        marker_point = self.point_from_detection(frame.depth, cone_det)
        object_point = self.point_from_detection(frame.depth, obj_det)

        marker_img_path = self.save_crop(
            image, cone_det.bbox, self.marker_dir, f"wp{wp_id}_attempt{attempt_idx}_marker.jpg"
        )
        object_img_path = self.save_crop(
            image, obj_det.bbox, self.object_dir, f"wp{wp_id}_attempt{attempt_idx}_object.jpg"
        )

        status = "ok"
        error_reason = ""
        distance = -1.0
        if marker_point is None or object_point is None:
            status = "partial"
            error_reason = "depth_unavailable"
        else:
            distance = float(np.linalg.norm(object_point - marker_point))
            cv2.putText(
                annotated,
                f"{obj_det.color} {obj_det.shape_name} {distance:.2f}m",
                (max(10, obj_det.bbox[0]), max(25, obj_det.bbox[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        annotated_img_path = self.save_annotated_image(
            annotated,
            cone_det,
            obj_det,
            wp_id,
            attempt_idx,
            status_text=status if not error_reason else error_reason,
        )

        result.update(
            {
                "status": status,
                "error_reason": error_reason,
                "obj_color": obj_det.color,
                "obj_class": obj_det.class_name,
                "obj_shape": obj_det.shape_name,
                "img_path": object_img_path,
                "marker_img_path": marker_img_path,
                "annotated_img_path": annotated_img_path,
            }
        )
        if marker_point is not None:
            result["marker_pose_camera"] = self.point_to_dict(marker_point)
        if object_point is not None:
            result["object_pose_camera"] = self.point_to_dict(object_point)
            result["dist"] = round(distance, 3)
        result["_score"] = cone_det.score + obj_det.score + (2.0 if status == "ok" else 0.25)
        return result

    def _base_result(self, wp_id: str, attempt_idx: int) -> Dict:
        return {
            "run_id": self.run_id,
            "wp_id": wp_id,
            "attempt_idx": attempt_idx,
            "status": "failed",
            "error_reason": "",
            "marker_color": "orange",
            "obj_color": "unknown",
            "obj_class": "unknown",
            "obj_shape": "unknown",
            "dist": -1.0,
            "distance_method": "depth_3d",
            "img_path": "",
            "marker_img_path": "",
            "annotated_img_path": "",
            "marker_pose_camera": None,
            "object_pose_camera": None,
            "ts": round(self.get_clock().now().nanoseconds / 1e9, 3),
        }

    def detect_cone(self, image: np.ndarray) -> Optional[Detection]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([5, 120, 80]), np.array([22, 255, 255]))
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = -1.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 200.0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < max(w, 15):
                continue
            score = area * min(2.0, h / max(1.0, w))
            if score > best_score:
                best = Detection(color="orange", contour=contour, bbox=(x, y, w, h), score=float(score))
                best_score = score
        return best

    def detect_object(self, image: np.ndarray, cone_bbox: Tuple[int, int, int, int]) -> Optional[Detection]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {
            "dark_green": cv2.inRange(hsv, np.array([35, 35, 25]), np.array([90, 255, 180])),
            "yellow": cv2.inRange(hsv, np.array([18, 90, 90]), np.array([38, 255, 255])),
            "red": cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255])),
                cv2.inRange(hsv, np.array([170, 90, 70]), np.array([180, 255, 255])),
            ),
            "orange": cv2.inRange(hsv, np.array([5, 120, 80]), np.array([22, 255, 255])),
            "blue": cv2.inRange(hsv, np.array([90, 70, 50]), np.array([130, 255, 255])),
        }

        best = None
        best_score = -1.0
        cone_rect = self.rect_to_box(cone_bbox)
        for color_name, mask in masks.items():
            mask = cv2.medianBlur(mask, 5)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 300.0:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                bbox = (x, y, w, h)
                if self.iou(cone_rect, self.rect_to_box(bbox)) > 0.25:
                    continue
                class_name, shape_name = self.classify_object(contour)
                score = float(area)
                if class_name != "unknown":
                    score *= 1.2
                if score > best_score:
                    best = Detection(
                        color=color_name,
                        contour=contour,
                        bbox=bbox,
                        score=score,
                        class_name=class_name,
                        shape_name=shape_name,
                    )
                    best_score = score
        return best

    def classify_object(self, contour: np.ndarray) -> Tuple[str, str]:
        peri = cv2.arcLength(contour, True)
        if peri <= 0.0:
            return "unknown", "unknown"

        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)
        area = max(cv2.contourArea(contour), 1e-6)
        circularity = 4.0 * math.pi * area / max(peri * peri, 1e-6)

        if vertices == 4:
            _, _, w, h = cv2.boundingRect(approx)
            ratio = w / max(float(h), 1e-6)
            if 0.85 <= ratio <= 1.15:
                return "cube", "square"
            return "cuboid", "rectangle"
        if circularity > 0.72:
            return "cylinder", "circle"
        return "unknown", "unknown"

    def point_from_detection(self, depth_img: Optional[np.ndarray], detection: Detection) -> Optional[np.ndarray]:
        if not self.use_depth or depth_img is None or not self.camera_info_ready:
            return None

        mask = np.zeros(depth_img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [detection.contour], contourIdx=-1, color=255, thickness=-1)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
        valid = depth_img[mask > 0].astype(np.float32)
        valid = valid[np.isfinite(valid)]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            x, y, w, h = detection.bbox
            u = int(x + w / 2.0)
            v = int(y + h / 2.0)
            valid = self.depth_patch(depth_img, u, v)
            if valid is None:
                return None
            depth = valid
        else:
            depth = float(np.median(valid))
            if depth > 20.0:
                depth /= 1000.0

        x, y, w, h = detection.bbox
        u = x + w / 2.0
        v = y + h / 2.0
        px = (u - self.cx) * depth / self.fx
        py = (v - self.cy) * depth / self.fy
        return np.array([float(px), float(py), float(depth)], dtype=np.float32)

    def depth_patch(self, depth_img: np.ndarray, u: int, v: int, window: int = 5) -> Optional[float]:
        h, w = depth_img.shape[:2]
        x1 = max(0, int(u) - window)
        x2 = min(w, int(u) + window + 1)
        y1 = max(0, int(v) - window)
        y2 = min(h, int(v) + window + 1)
        patch = depth_img[y1:y2, x1:x2].astype(np.float32)
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None
        depth = float(np.median(valid))
        if depth > 20.0:
            depth /= 1000.0
        return depth

    @staticmethod
    def point_to_dict(point: np.ndarray) -> Dict[str, float]:
        return {"x": round(float(point[0]), 4), "y": round(float(point[1]), 4), "z": round(float(point[2]), 4)}

    def save_crop(self, image: np.ndarray, bbox: Tuple[int, int, int, int], directory: Path, filename: str) -> str:
        path = directory / filename
        if self.save_raw:
            x, y, w, h = bbox
            margin = 12
            x1 = max(0, x - margin)
            y1 = max(0, y - margin)
            x2 = min(image.shape[1], x + w + margin)
            y2 = min(image.shape[0], y + h + margin)
            cv2.imwrite(str(path), image[y1:y2, x1:x2])
            return str(path)
        return ""

    def save_annotated_image(
        self,
        annotated: np.ndarray,
        cone_det: Detection,
        obj_det: Optional[Detection],
        wp_id: str,
        attempt_idx: int,
        status_text: str,
    ) -> str:
        cv2.putText(
            annotated,
            f"wp={wp_id} attempt={attempt_idx} {status_text}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        self.draw_detection(annotated, cone_det, label="marker")
        if obj_det is not None:
            self.draw_detection(annotated, obj_det, label=f"{obj_det.color} {obj_det.shape_name}")
        path = self.annotated_dir / f"wp{wp_id}_attempt{attempt_idx}_annotated.jpg"
        if self.save_annotated:
            cv2.imwrite(str(path), annotated)
            return str(path)
        return ""

    @staticmethod
    def draw_detection(image: np.ndarray, detection: Detection, label: str) -> None:
        x, y, w, h = detection.bbox
        color = (0, 255, 0) if detection.color != "orange" else (0, 140, 255)
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        cv2.putText(
            image,
            label,
            (x, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    @staticmethod
    def rect_to_box(bbox: Tuple[int, int, int, int]) -> Tuple[int, int, int, int]:
        x, y, w, h = bbox
        return x, y, x + w, y + h

    @staticmethod
    def iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
        area_b = max(1, (bx2 - bx1) * (by2 - by1))
        return inter / float(area_a + area_b - inter)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
