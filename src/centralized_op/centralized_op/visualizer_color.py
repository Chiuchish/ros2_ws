import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import json
import math

class MultiRobotVisualizer(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        # ==========================================
        # 1. 嚴格參數宣告 (符合 ROS 2 Jazzy 規範)
        # ==========================================
        self.declare_parameter('num_robots', value=20)
        self.declare_parameter('speed_threshold', value=1.0) # 區分快/慢車的速度門檻值 (m/s)
        self.declare_parameter('k_nearest', value=2)         # 動態連線繪製的鄰居數
        self.declare_parameter('target_center_x', value=0.0)
        self.declare_parameter('target_center_y', value=0.0)
        
        # 透過 JSON 字串傳入每台車的屬性配置: {"0": {"max_speed": 1.2, "sensor_quality": "Good"}, ...}
        self.declare_parameter('robot_configs_json', value='{}')

        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('slot_offsets_x', value=[0.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0], descriptor=array_desc)

        # 讀取參數
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.speed_threshold = float(self.get_parameter('speed_threshold').value)
        self.k_nearest = int(self.get_parameter('k_nearest').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        self.target_center_y = float(self.get_parameter('target_center_y').value)
        
        raw_configs = self.get_parameter('robot_configs_json').value
        try:
            self.robot_configs = json.loads(raw_configs)
        except Exception:
            self.robot_configs = {}

        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value)
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value)

        # 儲存小車即時位置: {robot_id: (x, y)}
        self.robot_positions = {}

        # ==========================================
        # 2. 訂閱所有小車的座標狀態
        # ==========================================
        for i in range(self.num_robots):
            topic_name = f'/robot_{i}/kinematic_state'
            self.create_subscription(
                Float32MultiArray,
                topic_name,
                lambda msg, r_id=i: self.state_callback(msg, r_id),
                10
            )

        # ==========================================
        # 3. 初始化 Matplotlib 畫布
        # ==========================================
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(9, 8))
        self.setup_legend()

        # 啟動 30Hz 高速繪圖更新計時器
        self.timer = self.create_timer(0.033, self.draw_callback)

    def state_callback(self, msg, robot_id):
        # msg.data: [x, y, offset_x, offset_y]
        if len(msg.data) >= 2:
            self.robot_positions[robot_id] = (msg.data[0], msg.data[1])

    def get_robot_color(self, robot_id):
        """
        核心能力顏色判定：
        - 紅色: 感測器差 (Poor / bad)
        - #橘色: 慢車 (速度小於門檻)
        - 藍色: 感測器好 
        """
        # 優先從 launch 傳入的配置讀取，若無則預設為 Good / 1.0 m/s
        config = self.robot_configs.get(str(robot_id), {})
        sensor = config.get('sensor_quality', 'Good')
        speed = float(config.get('max_speed', 1.0))

        # 1. 優先檢查感測器品質 (最高警示)
        if sensor in ['Poor', 'poor', 'bad', 'Bad']:
            return '#E74C3C'  # 鮮明紅 (Faulty Sensor)
        # 3. 快車
        else:
            return '#2980B9'  # 科技藍 (Fast Agent)

    def setup_legend(self):
        """建立圖例說明，展示給審查者/教授時更直觀"""
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', label='Poor Positioning Robot', markerfacecolor='#E74C3C', markersize=15),
            #Line2D([0], [0], marker='o', color='w', label=f'Slow Robot (<{self.speed_threshold}m/s)', markerfacecolor='#E67E22', markersize=15),
            Line2D([0], [0], marker='o', color='w', label='Good Positioning Robot', markerfacecolor='#2980B9', markersize=15),
            #Line2D([0], [0], marker='x', color='gray', label='Target Slots', linestyle='None', markersize=8),
            #Line2D([0], [0], color='gray', linestyle='--', alpha=0.6, label='K-NN Topology')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', framealpha=0.9, fontsize=12)

    def draw_callback(self):
        if not plt.fignum_exists(self.fig.number):
            return

        self.ax.clear()
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_title(f"Heterogeneous Multi-Robot System (N={self.num_robots})", fontsize=17)
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")

        # 1. 繪製最終目標 Slot 位置 (以灰色叉叉表示)
        for ox, oy in zip(self.slot_offsets_x, self.slot_offsets_y):
            self.ax.plot(self.target_center_x + ox, self.target_center_y + oy, 'kx', alpha=0.4, markersize=8)

        # 2. 繪製動態 K-NN 拓撲連線 (虛擬彈簧線)
        for r_id, pos in self.robot_positions.items():
            dist_list = []
            for other_id, other_pos in self.robot_positions.items():
                if r_id != other_id:
                    d = math.hypot(pos[0] - other_pos[0], pos[1] - other_pos[1])
                    dist_list.append((d, other_pos))
            
            dist_list.sort(key=lambda item: item[0])
            for _, neighbor_pos in dist_list[:self.k_nearest]:
                self.ax.plot([pos[0], neighbor_pos[0]], [pos[1], neighbor_pos[1]], 
                             color='white', linestyle='--', alpha=0.3, linewidth=1.0)

        # 3. 繪製小車本體 (依照能力套用顏色)
        for r_id, (x, y) in self.robot_positions.items():
            color = self.get_robot_color(r_id)
            self.ax.scatter(x, y, color=color, s=120, edgecolors='black', linewidth=1.2, zorder=5)
            self.ax.text(x + 0.1, y + 0.1, f"R{r_id}", fontsize=9, fontweight='bold', zorder=6)

        # 動態調整邊界
        """if self.robot_positions:
            all_x = [p[0] for p in self.robot_positions.values()]
            all_y = [p[1] for p in self.robot_positions.values()]
            self.ax.set_xlim(min(all_x) - 1.5, max(all_x) + 1.5)
            self.ax.set_ylim(min(all_y) - 1.5, max(all_y) + 1.5)"""
        self.ax.set_xlim(-10.0, 10.0)
        self.ax.set_ylim(-10.0, 10.0)
        self.setup_legend()
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()