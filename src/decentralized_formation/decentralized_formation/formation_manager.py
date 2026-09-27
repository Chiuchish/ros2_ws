import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import math
import numpy as np

class FormationManagerNode(Node):
    def __init__(self):
        super().__init__('formation_manager_node')

        # 1. 嚴格參數宣告 (ROS 2 Jazzy 規範)
        self.declare_parameter('num_robots', 26)
        self.declare_parameter('target_center_x', 0.0)
        self.declare_parameter('target_center_y', 0.0)
        self.declare_parameter('auto_switch', False)          # 是否定時自動變換圖形
        self.declare_parameter('switch_interval', 10.0)       # 自動變換週期 (秒)
        self.declare_parameter('default_formation', 'grid')   # 初始圖形

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.center_x = float(self.get_parameter('target_center_x').value)
        self.center_y = float(self.get_parameter('target_center_y').value)
        self.auto_switch = bool(self.get_parameter('auto_switch').value)
        self.switch_interval = float(self.get_parameter('switch_interval').value)

        # 2. 狀態變數
        self.formation_version = 0
        self.available_formations = ['grid', 'circle', 'wedge', 'column']
        self.current_index = 0

        # 3. 發布者與訂閱者
        # 發布陣型 Slot 座標資訊
        self.formation_pub = self.create_publisher(String, '/formation_slots_config', 10)
        
        # 訂閱外部圖形變換指令 (例如: ros2 topic pub /formation_cmd ...)
        self.create_subscription(String, '/formation_cmd', self.cmd_callback, 10)

        # 定時自動切換陣型
        if self.auto_switch:
            self.switch_timer = self.create_timer(self.switch_interval, self.auto_switch_callback)

        # 啟動時發送初始圖形 (延遲 1 秒確保其餘節點已上線)
        self.init_timer = self.create_timer(1.0, self.publish_initial_formation)
        self.get_logger().info(f"📐 [Formation Manager] 編隊圖形管理器已啟動，管控 {self.num_robots} 台小車之陣型圖庫。")

    # ==========================================
    # 陣型幾何演算法生成庫 (支援任意規模與 26 機)
    # ==========================================
    def generate_grid_formation(self, spacing=1.2):
        """方陣 (Grid): 近似對稱的 5 列網格"""
        offsets_x, offsets_y = [], []
        cols = 5
        rows = math.ceil(self.num_robots / cols)
        
        start_x = -((cols - 1) * spacing) / 2.0
        start_y = -((rows - 1) * spacing) / 2.0

        count = 0
        for r in range(rows):
            for c in range(cols):
                if count >= self.num_robots:
                    break
                offsets_x.append(round(start_x + c * spacing, 3))
                offsets_y.append(round(start_y + r * spacing, 3))
                count += 1
        return offsets_x, offsets_y

    def generate_circle_formation(self, radius=3.2):
        """環形 (Circle): 均勻分佈於半徑 R 的圓周上"""
        offsets_x, offsets_y = [], []
        angle_step = (2 * math.pi) / self.num_robots
        
        for i in range(self.num_robots):
            angle = i * angle_step
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            offsets_x.append(round(x, 3))
            offsets_y.append(round(y, 3))
        return offsets_x, offsets_y

    def generate_wedge_formation(self, spacing=1.0, angle_deg=60):
        """楔形/V字陣 (Wedge): 尖端領頭，兩翼等間距展開"""
        offsets_x, offsets_y = [], []
        theta = math.radians(angle_deg / 2.0)

        # 尖端 (Slot 0)
        offsets_x.append(0.0)
        offsets_y.append(2.0)

        # 左右兩翼展開
        left_count = (self.num_robots - 1) // 2
        right_count = (self.num_robots - 1) - left_count

        for i in range(1, left_count + 1):
            offsets_x.append(round(-i * spacing * math.sin(theta), 3))
            offsets_y.append(round(2.0 - i * spacing * math.cos(theta), 3))

        for i in range(1, right_count + 1):
            offsets_x.append(round(i * spacing * math.sin(theta), 3))
            offsets_y.append(round(2.0 - i * spacing * math.cos(theta), 3))

        return offsets_x, offsets_y

    def generate_column_formation(self, row_spacing=1.2, col_spacing=2.5):
        """雙縱列 (Double Column): 兩條平行縱隊，適合狹窄通道巡航"""
        offsets_x, offsets_y = [], []
        half = self.num_robots // 2
        
        # 左列
        for i in range(half):
            offsets_x.append(round(-col_spacing / 2.0, 3))
            offsets_y.append(round((i - half / 2.0) * row_spacing, 3))

        # 右列
        for i in range(half, self.num_robots):
            offsets_x.append(round(col_spacing / 2.0, 3))
            offsets_y.append(round((i - half - half / 2.0) * row_spacing, 3))

        return offsets_x, offsets_y

    # ==========================================
    # 變換調度與發布邏輯
    # ==========================================
    def switch_to_formation(self, shape_name: str):
        shape_name = shape_name.lower().strip()

        if shape_name == 'grid':
            ox, oy = self.generate_grid_formation()
        elif shape_name == 'circle':
            ox, oy = self.generate_circle_formation()
        elif shape_name == 'wedge':
            ox, oy = self.generate_wedge_formation()
        elif shape_name == 'column':
            ox, oy = self.generate_column_formation()
        else:
            self.get_logger().warn(f"未知陣型名稱 '{shape_name}'，保持原陣型。")
            return

        self.formation_version += 1
        payload = {
            'formation_version': self.formation_version,
            'formation_name': shape_name,
            'target_center_x': self.center_x,
            'target_center_y': self.center_y,
            'num_slots': len(ox),
            'slot_offsets_x': ox,
            'slot_offsets_y': oy
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.formation_pub.publish(msg)

        self.get_logger().info(
            f"🚀 [Formation Trigger] 切換至陣型: 【{shape_name.upper()}】 "
            f"(版本號: v{self.formation_version}, 總槽位數: {len(ox)})"
        )

    def publish_initial_formation(self):
        default_shape = self.get_parameter('default_formation').value
        self.switch_to_formation(default_shape)
        self.init_timer.cancel()

    def auto_switch_callback(self):
        self.get_logger().info(f"💱 [Formation Manager] Formation switch")
        self.current_index = (self.current_index + 1) % len(self.available_formations)
        next_shape = self.available_formations[self.current_index]
        self.switch_to_formation(next_shape)

    def cmd_callback(self, msg):
        requested_shape = msg.data.strip()
        self.switch_to_formation(requested_shape)

def main(args=None):
    rclpy.init(args=args)
    node = FormationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()