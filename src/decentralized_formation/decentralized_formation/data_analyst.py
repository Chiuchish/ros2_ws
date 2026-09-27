import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import json
import math
import os

class DistanceEvaluatorNode(Node):
    def __init__(self):
        super().__init__('distance_evaluator_node')

# ========================================================
        # 步驟 1：必須先宣告並取得參數 (確保 self.num_robots 存在)
        # ========================================================
        self.declare_parameter('num_robots', value=26)
        self.declare_parameter('robot_configs_json', value='{}')
        self.declare_parameter('output_path', value='sensor_distance_comparison.png')

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.output_path = str(self.get_parameter('output_path').value)
        
        raw_configs = self.get_parameter('robot_configs_json').value
        try:
            self.robot_configs = json.loads(raw_configs)
        except Exception:
            self.robot_configs = {}

        # ========================================================
        # 步驟 2：初始化統計變數與收斂緩衝區 (這時 self.num_robots 才能安全使用)
        # ========================================================
        self.last_positions = {}
        self.travel_distances = {i: 0.0 for i in range(self.num_robots)}
        self.is_moving = {i: True for i in range(self.num_robots)}

        # 收斂檢測與滑動窗口緩衝區
        self.is_arrived = {i: False for i in range(self.num_robots)}
        self.target_slots = {}

        # ========================================================
        # 步驟 3：訂閱話題與設定 Timer
        # ========================================================
        for i in range(self.num_robots):
            topic = f'/robot_{i}/kinematic_state'
            self.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg, r_id=i: self.state_callback(msg, r_id),
                10
            )

        self.check_timer = self.create_timer(5.0, self.check_convergence_and_log)
        self.get_logger().info(f"📊 距離評估節點已啟動，監控 {self.num_robots} 台小車...")

    def state_callback(self, msg, robot_id):
        # 1. 基本防呆：若已到達或資料長度不足則直接略過
        if self.is_arrived[robot_id] or len(msg.data) < 2:
            return

        curr_x = float(msg.data[0])
        curr_y = float(msg.data[1])

        # 2. 取得目標 Slot 座標 (依據小車發布的 [x, y, target_x, target_y])
        if len(msg.data) >= 4:
            self.target_slots[robot_id] = (float(msg.data[2]), float(msg.data[3]))

        # 若尚未取得目標點分配，只更新座標並返回
        if robot_id not in self.target_slots:
            self.last_positions[robot_id] = (curr_x, curr_y)
            return

        target_x, target_y = self.target_slots[robot_id]

        # 3. 計算與目標點的歐幾里得距離
        dist_to_target = math.hypot(curr_x - target_x, curr_y - target_y)

        # 4. 依照感測器好壞給定到達門檻 (Poor 車放寬以防死鎖)
        cfg = self.robot_configs.get(str(robot_id), {})
        sensor = cfg.get('sensor_quality', 'Good')
        tolerance = 0.25 if sensor in ['Poor', 'poor', 'bad', 'Bad'] else 0.10

        # 5. 到達判定
        if dist_to_target <= tolerance:
            self.is_arrived[robot_id] = True
            self.get_logger().info(
                f"🎯 Robot {robot_id} ({sensor}) 到達目標！殘餘誤差: {dist_to_target:.3f} m"
            )
            return

        # 6. 未到達前，正常累加行駛距離
        if robot_id in self.last_positions:
            prev_x, prev_y = self.last_positions[robot_id]
            delta = math.hypot(curr_x - prev_x, curr_y - prev_y)
            if delta > 1e-4:
                self.travel_distances[robot_id] += delta

        self.last_positions[robot_id] = (curr_x, curr_y)

    def check_convergence_and_log(self):
        arrived_count = sum(1 for status in self.is_arrived.values() if status)
        self.get_logger().info(f"📊 收斂進度: [{arrived_count}/{self.num_robots}] 台小車已到位")

        # 全體小車皆進入目標半徑時自動產出箱形圖
        if arrived_count == self.num_robots:
            self.get_logger().info("✅ 全員皆已精準抵達目標 Slot，正在生成箱形圖...")
            self.generate_boxplot()
            self.check_timer.cancel()

    def generate_boxplot(self):
        """將數據分類並繪製 Box Plot"""
        good_distances = []
        poor_distances = []

        for r_id, dist in self.travel_distances.items():
            cfg = self.robot_configs.get(str(r_id), {})
            sensor = cfg.get('sensor_quality', 'Good')

            if sensor in ['Poor', 'poor', 'bad', 'Bad']:
                poor_distances.append(dist)
            else:
                good_distances.append(dist)

        if not good_distances and not poor_distances:
            self.get_logger().warn("無足夠行駛距離數據，略過繪圖。")
            return

        # 繪圖設定 (適合論文/專題報告排版)
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
        fig, ax = plt.subplots(figsize=(7, 6))

        data = [good_distances, poor_distances]
        labels = [f'Good Positioning\n(N={len(good_distances)})', f'Poor Positioning\n(N={len(poor_distances)})']
        colors = ['#2980B9', '#E74C3C']  # 藍色 vs 紅色

        box = ax.boxplot(
            data,
            labels=labels,       # <-- 將原本的 tick_labels=labels 改為 labels=labels
            patch_artist=True,
            widths=0.45,
            medianprops=dict(color='black', linewidth=1.5),
            whiskerprops=dict(color='gray', linewidth=1.2),
            capprops=dict(color='gray', linewidth=1.2),
            flierprops=dict(marker='o', color='gray', alpha=0.5)
        )

        # 填色與散點疊加 (Jitter Plot 凸顯單機離散程度)
        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

        """for idx, dists in enumerate(data, start=1):
            jitter = np.random.normal(idx, 0.04, size=len(dists))
            ax.scatter(jitter, dists, color=colors[idx-1], edgecolor='black', s=45, alpha=0.85, zorder=3)"""

        ax.set_ylabel('Total Traveled Distance (m)', fontsize=12, fontweight='bold')
        ax.set_title('Traveled Distance Comparison by Positioning Quality', fontsize=13, fontweight='bold')
        ax.grid(axis='y', linestyle='--', alpha=0.7)

        # 儲存圖片
        plt.tight_layout()
        plt.savefig(self.output_path, dpi=300)
        self.get_logger().info(f"✅ 箱形圖已成功儲存至: {os.path.abspath(self.output_path)}")
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = DistanceEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中斷訊號，正在生成最終統計箱形圖...")
    finally:
        node.generate_boxplot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()