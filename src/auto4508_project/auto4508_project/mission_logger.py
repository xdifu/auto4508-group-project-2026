import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
import os

class MissionLoggerNode(Node):
    def __init__(self):
        super().__init__('mission_logger')
        
        # 1. 创建日志目录
        self.start_time = datetime.now()
        self.run_id = self.start_time.strftime("run_%Y%m%d_%H%M%S")
        self.log_dir = os.path.expanduser(f"~/AUTO4508_Logs/{self.run_id}")
        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(f"📁 Logger Started. Data will be saved to: {self.log_dir}")

        # 2. 核心内存数据库
        self.logs = [] 

        # 3. 订阅话题
        self.create_subscription(String, '/mission/navigation', self.nav_cb, 10)
        self.create_subscription(String, '/mission/vision', self.vision_cb, 10)
        self.create_subscription(String, '/mission/safety', self.safety_cb, 10)

    # --- 统一的 JSON 解析与记录器 ---
    def parse_and_record(self, category, msg_data):
        try:
            data = json.loads(msg_data)
            # 加入精确的时间戳
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            log_entry = {
                "timestamp": timestamp, 
                "category": category, 
                "data": data
            }
            self.logs.append(log_entry)
            self.get_logger().info(f"[{category}] Event logged at {timestamp}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON received on {category}")

    # --- 频道回调 ---
    def nav_cb(self, msg): self.parse_and_record("NAV", msg.data)
    def vision_cb(self, msg): self.parse_and_record("VISION", msg.data)
    def safety_cb(self, msg): self.parse_and_record("SAFETY", msg.data)

    # ==========================================
    # 终极奥义：退出时生成 JSON 和双版本 HTML
    # ==========================================
    def generate_outputs(self):
        end_time = datetime.now()
        duration = (end_time - self.start_time).total_seconds()
        
        # ----------------------------------
        # 1. 导出最核心的 JSON 原始文件
        # ----------------------------------
        json_path = os.path.join(self.log_dir, "mission_data.json")
        with open(json_path, "w") as f:
            json.dump(self.logs, f, indent=4, ensure_ascii=False)

        # ----------------------------------
        # 2. 统计数据 (为 Summary HTML 准备)
        # ----------------------------------
        nav_count = sum(1 for x in self.logs if x['category'] == 'NAV')
        vision_count = sum(1 for x in self.logs if x['category'] == 'VISION')
        safety_count = sum(1 for x in self.logs if x['category'] == 'SAFETY')

        # CSS 样式模板 (让网页看起来高大上)
        css_style = """
        <style>
            body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #1e1e1e; color: #f0f0f0; padding: 20px; }
            h1, h2 { color: #00d2ff; border-bottom: 1px solid #444; padding-bottom: 10px; }
            .card { background: #2a2a2a; border-radius: 8px; padding: 15px; margin: 10px 0; border-left: 5px solid #00d2ff; }
            .NAV { border-left-color: #00ff88; }
            .VISION { border-left-color: #bb00ff; }
            .SAFETY { border-left-color: #ff3333; }
            table { width: 100%; border-collapse: collapse; margin-top: 20px; }
            th, td { padding: 12px; text-align: left; border-bottom: 1px solid #444; }
            th { background-color: #333; }
            .timestamp { color: #888; font-family: monospace; }
        </style>
        """

        # ----------------------------------
        # 3. 生成 Summary HTML (总结大屏)
        # ----------------------------------
        summary_html = f"""
        <html><head><title>Mission Summary</title>{css_style}</head><body>
            <h1>AUTO4508 - Mission Summary Dashboard</h1>
            <p class="timestamp">Run ID: {self.run_id} | Total Duration: <b>{duration:.1f} sec</b></p>
            <div style="display: flex; gap: 20px; flex-wrap: wrap;">
                <div class="card NAV" style="flex: 1;"><h2>{nav_count}</h2><p>Waypoints Reached</p></div>
                <div class="card VISION" style="flex: 1;"><h2>{vision_count}</h2><p>Objects Detected</p></div>
                <div class="card SAFETY" style="flex: 1;"><h2>{safety_count}</h2><p>Safety Interventions</p></div>
            </div>
            <p style="margin-top:30px;"><a href="detailed_report.html" style="color:#00d2ff;">View Detailed Log &rarr;</a></p>
        </body></html>
        """
        with open(os.path.join(self.log_dir, "summary_dashboard.html"), "w", encoding='utf-8') as f:
            f.write(summary_html)

        # ----------------------------------
        # 4. 生成 Detailed HTML (详细时间轴)
        # ----------------------------------
        table_rows = ""
        for entry in self.logs:
            t = entry['timestamp']
            cat = entry['category']
            d = entry['data']
            
            # 将 JSON 字典美化为字符串展示
            detail_str = "<br>".join([f"<b>{k}</b>: {v}" for k, v in d.items()])
            
            table_rows += f"<tr><td class='timestamp'>{t}</td><td><span class='card {cat}' style='padding:4px 8px; font-weight:bold;'>{cat}</span></td><td>{detail_str}</td></tr>"

        detailed_html = f"""
        <html><head><title>Detailed Log</title>{css_style}</head><body>
            <h1>AUTO4508 - Detailed Event Log</h1>
            <p><a href="summary_dashboard.html" style="color:#00d2ff;">&larr; Back to Summary</a></p>
            <table>
                <tr><th>Timestamp</th><th>Category</th><th>Event Details</th></tr>
                {table_rows}
            </table>
        </body></html>
        """
        with open(os.path.join(self.log_dir, "detailed_report.html"), "w", encoding='utf-8') as f:
            f.write(detailed_html)
        # 把原先的 self.get_logger().info 换成下面这句
        print(f"✅ Mission finished. All files (JSON + HTMLs) saved to {self.log_dir}")
        self.get_logger().info(f"✅ Mission finished. All files (JSON + HTMLs) saved to {self.log_dir}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_outputs()  # 退出前生成报表
        node.destroy_node()
        # 只有在还没关闭的情况下，才调用 shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()