import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import json
import math
import random

class FormationControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        
        # 1. 嚴格參數宣告 (ROS 2 Jazzy 規範)
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_quality', 'Good')
        self.declare_parameter('num_robots', 26)  # 系統總車數
        self.declare_parameter('k_nearest', 2)    # 動態選擇最近的 k 台車作為拓撲鄰居
        self.declare_parameter('target_center_x', 0.0)
        self.declare_parameter('target_center_y', 0.0)
        self.declare_parameter('voltage', value=12.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('initial_value', value=[0.0, 0.0], descriptor=array_desc) 
        self.declare_parameter('slot_offsets_x', value=[0.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0], descriptor=array_desc)

        # 讀取參數
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.sensor_quality = str(self.get_parameter('sensor_quality').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.k_nearest = int(self.get_parameter('k_nearest').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        self.target_center_y = float(self.get_parameter('target_center_y').value)
        
        self.state = list(self.get_parameter('initial_value').value or [0.0, 0.0])
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value)
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value)
        self.voltage = float(self.get_parameter('voltage').value)
        self.battery_capacity = 2.2
        # 狀態與目標座標
        self.my_slot = -1  
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.target_x = None
        self.target_y = None
        self.neighbor_data = {} 
        self.loop_counter = 0 
        
        # 感測器漂移累積量 (僅針對 Poor 車款)
        self.accumulated_error_x = 0.0
        self.accumulated_error_y = 0.0

        # 異質硬體耗能係數設定
        self.p_base = 1.5  # 基礎底噪 1.5W
        self.p_sensor = 4.0 if self.sensor_quality == 'Good' else 0.8  # 高階感測器功耗較大
        self.k_v1 = 3.0    # 速度線性阻力係數
        self.k_v2 = 5.0    # 速度二次方電機耗能係數

        self.charge_rate = 0.08  # 充電速度: 0.08 V/s (每 10 秒約充回 0.8V)
        self.battery_max_limit = 12.10

        # 2. 訂閱當選 Auctioneer 發布的匈牙利指派名單
        self.create_subscription(
            String, 
            '/formation_assignment', 
            self.assignment_callback, 
            10
        )

        # 3. 發布運動狀態給 Visualizer 與 Distance Evaluator
        pub_topic = f'/robot_{self.robot_id}/kinematic_state'
        self.publisher_ = self.create_publisher(Float32MultiArray, pub_topic, 10)
        
        # 4. 訂閱所有其他小車的運動狀態，構建動態拓撲
        for j in range(self.num_robots):
            if j != self.robot_id:
                sub_topic = f'/robot_{j}/kinematic_state'
                self.create_subscription(
                    Float32MultiArray, sub_topic, 
                    lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
                )
            
        # 50Hz 高頻控制迴圈
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_control_loop)
        
        self.k_consensus = 0.2  # 鄰居隊形虛擬彈簧增益
        self.k_goal = 1.0       # 目標點收斂增益

        self.get_logger().info(f"🚗 [Motion Controller] Robot {self.robot_id} 控制節點就緒，等待任務指派...")

    def assignment_callback(self, msg):
        """接收拍賣者計算出的匈牙利指派結果與最新陣型座標"""
        try:
            data = json.loads(msg.data)
            winners = data['winners']
            
            # ★ 1. 立即同步最新的 Slot 偏移與中心點
            if 'slot_offsets_x' in data and 'slot_offsets_y' in data:
                self.slot_offsets_x = data['slot_offsets_x']
                self.slot_offsets_y = data['slot_offsets_y']
                self.target_center_x = data.get('target_center_x', self.target_center_x)
                self.target_center_y = data.get('target_center_y', self.target_center_y)

            # ★ 2. 更新自己的目標點（移除 if self.my_slot != slot_index 限制）
            if self.robot_id in winners:
                slot_index = winners.index(self.robot_id)
                self.my_slot = slot_index
                self.offset_x = float(self.slot_offsets_x[slot_index])
                self.offset_y = float(self.slot_offsets_y[slot_index])
                self.target_x = self.target_center_x + self.offset_x
                self.target_y = self.target_center_y + self.offset_y

                self.get_logger().info(
                    f"🎯 Robot {self.robot_id} 鎖定 Slot {slot_index} -> 目標: ({self.target_x:.2f}, {self.target_y:.2f}) 電量: {self.voltage}"
                )
        except Exception as e:
            self.get_logger().error(f"解析分配結果失敗: {e}")

    def neighbor_callback(self, msg, neighbor_id):
        # 儲存鄰居回報的最新狀態
        self.neighbor_data[neighbor_id] = msg.data

    def update_control_loop(self):
        self.loop_counter += 1
        
        # 若尚未收到分配結果，只在原地廣播自身座標，不產生控制引力
        if self.my_slot == -1 or self.target_x is None:
            self.broadcast_state()
            return

        # 模擬本機感測座標（若感測器差則疊加累積隨機漫步漂移）
        perceived_x = self.state[0]
        perceived_y = self.state[1]

        if self.sensor_quality in ['Poor', 'poor', 'bad', 'Bad']:
            self.accumulated_error_x += random.gauss(0.0, 0.002)
            self.accumulated_error_y += random.gauss(0.0, 0.002)
            perceived_x += self.accumulated_error_x
            perceived_y += self.accumulated_error_y

        # 1. 前往指派 Slot 的目標引力
        u_x = -self.k_goal * (perceived_x - self.target_x)
        u_y = -self.k_goal * (perceived_y - self.target_y)
        
        # 2. 動態 K-NN 拓撲引力 (只與同樣已有目標的鄰居產生拉扯)
        distance_list = []
        for j, data_j in self.neighbor_data.items():
            if len(data_j) >= 4:  # 確保鄰居也已經獲得目標 Slot
                x_j = float(data_j[0])
                y_j = float(data_j[1])
                dist = math.hypot(perceived_x - x_j, perceived_y - y_j)
                distance_list.append((dist, j, data_j))
            
        distance_list.sort(key=lambda item: item[0])
        active_neighbors = distance_list[:self.k_nearest]
        
        # 計算編隊一致性拉力
        for dist, j, data_j in active_neighbors:
            x_j = float(data_j[0])
            y_j = float(data_j[1])
            target_x_j = float(data_j[2])
            target_y_j = float(data_j[3])
            
            # (p_i - p_j) - (target_i - target_j)
            u_x -= self.k_consensus * ((perceived_x - x_j) - (self.target_x - target_x_j))
            u_y -= self.k_consensus * ((perceived_y - y_j) - (self.target_y - target_y_j))
            
        # 3. 速度飽和限制 (異質最高速度約束)
        speed_mag = math.hypot(u_x, u_y)
        if speed_mag > self.max_speed and speed_mag > 0.0:
            scale = self.max_speed / speed_mag
            u_x *= scale
            u_y *= scale

        # Battery 1. 計算瞬時機械運動功率
        p_motion = (self.k_v1 * speed_mag) + (self.k_v2 * (speed_mag ** 2))

        # Battery 2. 計算感測器劣化帶來的控制抖動額外功耗 (Jitter Loss)
        p_jitter = 0.0
        if self.sensor_quality in ['Poor', 'poor', 'bad', 'Bad']:
            # 當車速極低但在目標點附近顫動時，電機急開急停產生損耗
            p_jitter = 1.2 * abs(random.gauss(0.0, 0.5))

        # Battery 3. 瞬時總功率 (W)
        p_total = self.p_base + self.p_sensor + p_motion + p_jitter

        # Battery 4. 數值積分計算電壓下降量 (dt = self.timer_period)
        # dV = (P * dt) / (Capacity * 3600 * V_nom) * 全幅電壓跨度 (約 1.6V)
        dv = (p_total * self.timer_period) / (self.battery_capacity * 3600.0 * 11.1) * 1.6
        self.voltage = max(10.9, self.voltage - dv)        

        # 4. 運動學積分更新真實座標
        self.state[0] += u_x * self.timer_period
        self.state[1] += u_y * self.timer_period

        """
        current_speed = math.hypot(u_x, u_y)
        dist_to_goal = math.hypot(self.state[0] - self.target_x, self.state[1] - self.target_y) if self.target_x is not None else 99.0

        in_charging_zone = (self.state[1] <= -3.5) and (dist_to_goal < 0.3)

        if in_charging_zone:
            # 1. 待命充電模式：電壓上升
            if self.voltage < self.battery_max_limit:
                self.voltage = min(self.battery_max_limit, self.voltage + self.charge_rate * self.timer_period)
                if self.loop_counter % 100 == 0:  # 每 2 秒提示一次
                    self.get_logger().info(f"⚡ Robot {self.robot_id} 正在待命樁補電中... 當前電量: {self.voltage:.2f}V")
        else:
            # 2. 正常任務耗電模式 (物理功耗積分)
            p_total = self.p_base + self.p_sensor + (self.k_v1 * current_speed + self.k_v2 * (current_speed ** 2))
            dv = (p_total * self.timer_period) / (self.battery_capacity * 3600.0 * 11.1) * 1.6
            self.voltage = max(10.0, self.voltage - dv)
        """
            
        self.broadcast_state()
            
    def broadcast_state(self):
        msg = Float32MultiArray()
        tx = float(self.target_x) if self.target_x is not None else 0.0
        ty = float(self.target_y) if self.target_y is not None else 0.0
        
        # 資料格式：[0:當前x, 1:當前y, 2:目標x, 3:目標y, 4:即時電壓]
        msg.data = [
            float(self.state[0]),
            float(self.state[1]),
            tx,
            ty,
            float(self.voltage)
        ]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FormationControlAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()