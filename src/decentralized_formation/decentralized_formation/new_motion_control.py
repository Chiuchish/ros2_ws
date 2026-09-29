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
        
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_quality', 'Good')
        self.declare_parameter('num_robots', 26)
        self.declare_parameter('k_nearest', 2)
        self.declare_parameter('target_center_x', 0.0)
        self.declare_parameter('target_center_y', 0.0)
        self.declare_parameter('battery_capacity_ah', 2.2)
        self.declare_parameter('voltage', 12.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('initial_value', value=[0.0, 0.0], descriptor=array_desc) 
        self.declare_parameter('slot_offsets_x', value=[0.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0], descriptor=array_desc)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.sensor_quality = str(self.get_parameter('sensor_quality').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.k_nearest = int(self.get_parameter('k_nearest').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        self.target_center_y = float(self.get_parameter('target_center_y').value)
        self.battery_capacity = float(self.get_parameter('battery_capacity_ah').value)
        self.voltage = float(self.get_parameter('voltage').value)
        
        self.state = list(self.get_parameter('initial_value').value or [0.0, 0.0])
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value)
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value)
        
        # 狀態控制標記
        self.my_slot = -1  
        self.is_standby = False      # ★ 標記是否處於待命充電狀態
        self.target_x = None
        self.target_y = None
        self.neighbor_data = {} 
        self.loop_counter = 0 
        
        self.accumulated_error_x = 0.0
        self.accumulated_error_y = 0.0

        # 電池物理參數
        self.p_base = 1.5
        self.p_sensor = 4.0 if self.sensor_quality == 'Good' else 0.8
        self.k_v1 = 3.0
        self.k_v2 = 5.0
        self.charge_rate = 0.08      # 待命區補電速度 (V/s)
        self.battery_max_limit = 12.10
        
        # 訂閱全域匈牙利分配話題
        self.create_subscription(String, '/formation_assignment', self.assignment_callback, 10)

        # 廣播運動狀態
        pub_topic = f'/robot_{self.robot_id}/kinematic_state'
        self.publisher_ = self.create_publisher(Float32MultiArray, pub_topic, 10)
        
        # 訂閱所有其他小車狀態
        for j in range(self.num_robots):
            if j != self.robot_id:
                sub_topic = f'/robot_{j}/kinematic_state'
                self.create_subscription(
                    Float32MultiArray, sub_topic, 
                    lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
                )
            
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_control_loop)
        
        self.k_consensus = 0.5  
        self.k_goal = 1.5       

    def assignment_callback(self, msg):
        """解析拍賣分配：包含 20 個任務得標者與 6 個落選待命者"""
        try:
            data = json.loads(msg.data)
            winners = data.get('winners', [])
            unassigned = data.get('unassigned', [])
            
            # 同步最新幾何中心與槽位偏移
            if 'slot_offsets_x' in data and 'slot_offsets_y' in data:
                self.slot_offsets_x = data['slot_offsets_x']
                self.slot_offsets_y = data['slot_offsets_y']
                self.target_center_x = data.get('target_center_x', self.target_center_x)
                self.target_center_y = data.get('target_center_y', self.target_center_y)

            # ==========================================
            # 情況 1：入選場上編隊 (Winners)
            # ==========================================
            if self.robot_id in winners:
                slot_index = winners.index(self.robot_id)
                self.my_slot = slot_index
                self.is_standby = False
                self.target_x = self.target_center_x + float(self.slot_offsets_x[slot_index])
                self.target_y = self.target_center_y + float(self.slot_offsets_y[slot_index])
                self.get_logger().info(
                    f"🎯 [Motion] Robot {self.robot_id} 入選陣型 Slot {slot_index} -> 目標: ({self.target_x:.2f}, {self.target_y:.2f})"
                )

            # ==========================================
            # 情況 2：落選，退回待命充電區 (Unassigned)
            # ==========================================
            elif self.robot_id in unassigned:
                standby_idx = unassigned.index(self.robot_id)
                num_standby = len(unassigned)
                spacing = 1.2
                
                # 計算充電樁橫向均勻排列座標 (y = -5.0)
                start_x = -((num_standby - 1) * spacing) / 2.0
                self.target_x = start_x + (standby_idx * spacing)
                self.target_y = -10.0
                
                self.my_slot = -2         # 使用 -2 代表待命模式，避開 -1 的未指派中斷
                self.is_standby = True
                self.get_logger().warn(
                    f"💤 [Motion] Robot {self.robot_id} 進入待命退場！前往待命樁 {standby_idx} -> 目標: ({self.target_x:.2f}, {self.target_y:.2f})"
                )
            else:
                self.my_slot = -1
                self.target_x = None
                self.target_y = None

        except Exception as e:
            self.get_logger().error(f"解析分配失敗: {e}")

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbor_data[neighbor_id] = msg.data

    def update_control_loop(self):
        self.loop_counter += 1
        
        # ★ 修正防護：只有在尚未獲得任何目標點時才提早 return
        if self.target_x is None or self.target_y is None:
            self.broadcast_state()
            return

        perceived_x = self.state[0]
        perceived_y = self.state[1]

        if self.sensor_quality in ['Poor', 'poor', 'bad', 'Bad']:
            self.accumulated_error_x += random.gauss(0.0, 0.002)
            self.accumulated_error_y += random.gauss(0.0, 0.002)
            perceived_x += self.accumulated_error_x
            perceived_y += self.accumulated_error_y

        # 1. 前往目標位置的導航引力
        u_x = -self.k_goal * (perceived_x - self.target_x)
        u_y = -self.k_goal * (perceived_y - self.target_y)
        
        # 2. 拓撲共識引力（★ 關鍵隔離邏輯）
        # 只有「場上任務車輛」需要進行鄰居拓撲協同；待命小車單純自主導航，不拉扯隊形
        if not self.is_standby:
            distance_list = []
            for j, data_j in self.neighbor_data.items():
                # 確保鄰居資料完整，且該鄰居不是待命車輛 (待命車輛的 is_standby 標記為 1.0)
                if len(data_j) >= 6 and data_j[5] < 0.5:
                    x_j = float(data_j[0])
                    y_j = float(data_j[1])
                    dist = math.hypot(perceived_x - x_j, perceived_y - y_j)
                    distance_list.append((dist, j, data_j))
                
            distance_list.sort(key=lambda item: item[0])
            active_neighbors = distance_list[:self.k_nearest]
            
            for dist, j, data_j in active_neighbors:
                x_j = float(data_j[0])
                y_j = float(data_j[1])
                target_x_j = float(data_j[2])
                target_y_j = float(data_j[3])
                
                u_x -= self.k_consensus * ((perceived_x - x_j) - (self.target_x - target_x_j))
                u_y -= self.k_consensus * ((perceived_y - y_j) - (self.target_y - target_y_j))
            
        # 3. 速度飽和限制
        speed_mag = math.hypot(u_x, u_y)
        if speed_mag > self.max_speed and speed_mag > 0.0:
            scale = self.max_speed / speed_mag
            u_x *= scale
            u_y *= scale

        # 4. 運動學積分更新
        self.state[0] += u_x * self.timer_period
        self.state[1] += u_y * self.timer_period

        
        # 5. 電量物理模擬：待命充電 vs 巡航消耗
        dist_to_goal = math.hypot(self.state[0] - self.target_x, self.state[1] - self.target_y)
        in_charging_dock = self.is_standby and (self.state[1] <= -4.0) and (dist_to_goal < 0.35)

        if in_charging_dock:
            """
            # 在充電樁靜止補電
            if self.voltage < self.battery_max_limit:
                self.voltage = min(self.battery_max_limit, self.voltage + self.charge_rate * self.timer_period)
                if self.loop_counter % 150 == 0:
                    self.get_logger().info(f"⚡ Robot {self.robot_id} 正在待命樁補電: {self.voltage:.2f}V")
            """
        else:
            # 正常行駛或原地運算功耗
            p_total = self.p_base + self.p_sensor + (self.k_v1 * speed_mag + self.k_v2 * (speed_mag ** 2))
            dv = (p_total * self.timer_period) / (self.battery_capacity * 3600.0 * 11.1) * 1.6
            self.voltage = max(10.0, self.voltage - dv)
                
        self.broadcast_state()
            
    def broadcast_state(self):
        msg = Float32MultiArray()
        tx = float(self.target_x) if self.target_x is not None else 0.0
        ty = float(self.target_y) if self.target_y is not None else 0.0
        standby_flag = 1.0 if self.is_standby else 0.0
        
        # 資料欄位：[0:當前x, 1:當前y, 2:目標x, 3:目標y, 4:即時電壓, 5:是否為待命車輛]
        msg.data = [
            float(self.state[0]),
            float(self.state[1]),
            tx,
            ty,
            float(self.voltage),
            standby_flag
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