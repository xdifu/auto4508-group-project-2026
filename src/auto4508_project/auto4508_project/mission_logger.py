import json
import os
from html import escape
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String


class MissionLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("logger_node")

        self.declare_parameter("run_id", "part2_run")
        self.declare_parameter("artifacts_root", str(Path.cwd() / "artifacts" / "part2_runs" / "part2_run"))
        self.declare_parameter("navigation_topic", "/mission/navigation")
        self.declare_parameter("vision_topic", "/mission/vision")
        self.declare_parameter("safety_topic", "/mission/safety")
        self.declare_parameter("path_topic", "/driven_path")

        self.run_id = str(self.get_parameter("run_id").value)
        self.artifacts_root = Path(str(self.get_parameter("artifacts_root").value)).expanduser()
        if not self.artifacts_root.is_absolute():
            self.artifacts_root = Path.cwd() / self.artifacts_root
        self.logs_dir = self.artifacts_root / "logs"
        self.summary_dir = self.artifacts_root / "summary"
        self.map_dir = self.artifacts_root / "map"
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.summary_dir.mkdir(parents=True, exist_ok=True)
        self.map_dir.mkdir(parents=True, exist_ok=True)

        self.events: List[Dict] = []
        self.nav_events: List[Dict] = []
        self.vision_events: List[Dict] = []
        self.safety_events: List[Dict] = []
        self.waypoints: Dict[str, Dict] = {}
        self.path_points: List[tuple[float, float]] = []
        self.path_csv = self.summary_dir / "path.csv"

        self.create_subscription(String, str(self.get_parameter("navigation_topic").value), self.nav_cb, 20)
        self.create_subscription(String, str(self.get_parameter("vision_topic").value), self.vision_cb, 20)
        self.create_subscription(String, str(self.get_parameter("safety_topic").value), self.safety_cb, 20)
        self.create_subscription(PathMsg, str(self.get_parameter("path_topic").value), self.path_cb, 10)

        self.write_timer = self.create_timer(2.0, self.generate_outputs)
        self.get_logger().info(f"Logger writing artifacts to {self.artifacts_root}")

    def parse_json(self, payload: str) -> Dict:
        try:
            data = json.loads(payload)
            return data if isinstance(data, dict) else {}
        except json.JSONDecodeError:
            return {}

    def record_event(self, category: str, data: Dict) -> None:
        entry = {
            "ts": float(data.get("ts", self.get_clock().now().nanoseconds / 1e9)),
            "category": category,
            "data": data,
        }
        self.events.append(entry)

    def ensure_waypoint(self, wp_id: str) -> Dict:
        return self.waypoints.setdefault(
            wp_id,
            {
                "navigation": [],
                "vision": None,
                "verified": False,
                "failed": False,
            },
        )

    def nav_cb(self, msg: String) -> None:
        data = self.parse_json(msg.data)
        if not data:
            return
        self.nav_events.append(data)
        self.record_event("navigation", data)
        wp_id = str(data.get("wp_id", ""))
        if wp_id:
            waypoint = self.ensure_waypoint(wp_id)
            waypoint["navigation"].append(data)
            if data.get("phase") == "verified" and data.get("status") == "reached":
                waypoint["verified"] = True
            if data.get("status") == "failed":
                waypoint["failed"] = True

    def vision_cb(self, msg: String) -> None:
        data = self.parse_json(msg.data)
        if not data:
            return
        self.vision_events.append(data)
        self.record_event("vision", data)
        wp_id = str(data.get("wp_id", ""))
        if wp_id:
            waypoint = self.ensure_waypoint(wp_id)
            waypoint["vision"] = data

    def safety_cb(self, msg: String) -> None:
        data = self.parse_json(msg.data)
        if not data:
            return
        self.safety_events.append(data)
        self.record_event("safety", data)

    def path_cb(self, msg: PathMsg) -> None:
        self.path_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.write_path_png()

    def write_path_png(self) -> str:
        output = self.summary_dir / "path.png"
        canvas = np.full((600, 600, 3), 255, dtype=np.uint8)
        if not self.path_points:
            cv2.imwrite(str(output), canvas)
            return str(output)

        xs = [p[0] for p in self.path_points]
        ys = [p[1] for p in self.path_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min(520.0 / span_x, 520.0 / span_y)

        def project(pt):
            x = int(40 + (pt[0] - min_x) * scale)
            y = int(560 - (pt[1] - min_y) * scale)
            return x, y

        for idx in range(1, len(self.path_points)):
            cv2.line(canvas, project(self.path_points[idx - 1]), project(self.path_points[idx]), (0, 80, 220), 2)
        cv2.putText(canvas, self.run_id, (20, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 2)
        cv2.imwrite(str(output), canvas)
        return str(output)

    def summary_payload(self) -> Dict:
        return {
            "run_id": self.run_id,
            "navigation_events": len(self.nav_events),
            "vision_events": len(self.vision_events),
            "safety_events": len(self.safety_events),
            "waypoints": self.waypoints,
            "map_yaml": str((self.map_dir / "map.yaml")) if (self.map_dir / "map.yaml").exists() else "",
            "map_pgm": str((self.map_dir / "map.pgm")) if (self.map_dir / "map.pgm").exists() else "",
            "path_csv": str(self.path_csv) if self.path_csv.exists() else "",
            "path_png": str(self.summary_dir / "path.png"),
        }

    def image_tag(self, path: str, label: str) -> str:
        if not path:
            return "<span>missing</span>"
        candidate = Path(path)
        if not candidate.exists():
            return f"<span>missing: {escape(path)}</span>"
        return f'<img src="{escape(str(candidate))}" alt="{escape(label)}" style="max-width: 320px; border-radius: 6px;" />'

    def generate_outputs(self) -> None:
        mission_data = self.logs_dir / "mission_data.json"
        summary_json = self.summary_dir / "summary.json"
        summary_html = self.summary_dir / "summary.html"
        detailed_html = self.summary_dir / "detailed_report.html"

        with mission_data.open("w", encoding="utf-8") as handle:
            json.dump(self.events, handle, indent=2, ensure_ascii=False)
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(self.summary_payload(), handle, indent=2, ensure_ascii=False)

        path_png = self.write_path_png()
        map_pgm = self.map_dir / "map.pgm"

        waypoint_rows = []
        for wp_id, payload in sorted(self.waypoints.items(), key=lambda item: item[0]):
            vision = payload.get("vision") or {}
            waypoint_rows.append(
                f"""
                <tr>
                  <td>{escape(str(wp_id))}</td>
                  <td>{escape('yes' if payload.get('verified') else 'no')}</td>
                  <td>{escape(vision.get('obj_color', 'unknown'))}</td>
                  <td>{escape(vision.get('obj_class', 'unknown'))}</td>
                  <td>{escape(vision.get('obj_shape', 'unknown'))}</td>
                  <td>{escape(str(vision.get('dist', '')))}</td>
                  <td>{self.image_tag(vision.get('marker_img_path', ''), f'marker {wp_id}')}</td>
                  <td>{self.image_tag(vision.get('img_path', ''), f'object {wp_id}')}</td>
                  <td>{self.image_tag(vision.get('annotated_img_path', ''), f'annotated {wp_id}')}</td>
                </tr>
                """
            )

        summary_markup = f"""
        <html>
        <head>
          <title>AUTO4508 Part 2 Summary</title>
          <style>
            body {{ font-family: Arial, sans-serif; margin: 24px; background: #f4f6f8; color: #20242a; }}
            .card {{ background: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 2px 6px rgba(0,0,0,0.08); }}
            table {{ width: 100%; border-collapse: collapse; }}
            th, td {{ border-bottom: 1px solid #e2e8f0; padding: 10px; text-align: left; vertical-align: top; }}
            img {{ display: block; }}
            code {{ background: #eef2f7; padding: 2px 4px; border-radius: 4px; }}
          </style>
        </head>
        <body>
          <div class="card">
            <h1>AUTO4508 Part 2 Summary</h1>
            <p><b>Run ID:</b> {escape(self.run_id)}</p>
            <p><b>Navigation events:</b> {len(self.nav_events)} | <b>Vision events:</b> {len(self.vision_events)} | <b>Safety events:</b> {len(self.safety_events)}</p>
          </div>
          <div class="card">
            <h2>Driven Path</h2>
            {self.image_tag(path_png, 'path')}
          </div>
          <div class="card">
            <h2>Built Map</h2>
            {self.image_tag(str(map_pgm) if map_pgm.exists() else '', 'map')}
          </div>
          <div class="card">
            <h2>Waypoint Evidence</h2>
            <table>
              <tr>
                <th>WP</th><th>Verified</th><th>Color</th><th>Class</th><th>Shape</th><th>Distance (m)</th>
                <th>Marker</th><th>Object</th><th>Annotated</th>
              </tr>
              {''.join(waypoint_rows)}
            </table>
          </div>
          <div class="card">
            <h2>Safety Events</h2>
            <pre>{escape(json.dumps(self.safety_events[-20:], indent=2, ensure_ascii=False))}</pre>
          </div>
        </body>
        </html>
        """
        summary_html.write_text(summary_markup, encoding="utf-8")

        detailed_markup = f"""
        <html><head><title>Detailed Report</title></head>
        <body>
        <h1>Detailed Event Log</h1>
        <pre>{escape(json.dumps(self.events, indent=2, ensure_ascii=False))}</pre>
        </body></html>
        """
        detailed_html.write_text(detailed_markup, encoding="utf-8")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
