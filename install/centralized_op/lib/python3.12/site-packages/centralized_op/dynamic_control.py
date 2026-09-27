import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import json
import traceback
import math
import random

class FormationControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_quality', 'Good')
        
        # ==========================================
        # ★ 修改 1：移除 fixed neighbors，改為讀取總車數與 k 值
        # ==========================================
        self.declare_parameter('num_robots', 4)  # 系統總車數
        self.declare_parameter('k_nearest', 2)   # 動態選擇最近的 2 台車作為鄰居
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('initial_value', value=[0.0, 0.0], descriptor=array_desc) 
        self.declare_parameter('slot_offsets_x', value=[0.0, -1.0, -1.0, -2.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0, 1.0, -1.0, 0.0], descriptor=array_desc)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.sensor_quality = self.get_parameter('sensor_quality').value
        
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.k_nearest = int(self.get_parameter('k_nearest').value)
        
        self.state = list(self.get_parameter('initial_value').value or [0.0, 0.0])
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])
        
        self.my_slot = -1  
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.neighbor_data = {} 
        self.loop_counter = 0 
        
        self.accumulated_error_x = 0.0
        self.accumulated_error_y = 0.0
        
        self.create_subscription(String, f'/robot_{self.robot_id}/auction_state', self.auction_callback, 10)

        pub_topic = f'/robot_{self.robot_id}/kinematic_state'
        self.publisher_ = self.create_publisher(Float32MultiArray, pub_topic, 10)
        
        # ==========================================
        # ★ 修改 2：訂閱「除了自己以外」的所有小車狀態
        # ==========================================
        for j in range(self.num_robots):
            if j != self.robot_id:
                sub_topic = f'/robot_{j}/kinematic_state'
                self.create_subscription(
                    Float32MultiArray, sub_topic, 
                    lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
                )
            
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_control_loop)
        
        self.k_consensus = 1.0  
        self.k_goal = 0.5       

    def auction_callback(self, msg):
        try:
            data = json.loads(msg.data)
            winners = data['winners']
            is_consensus = data.get('is_consensus', False)
            
            if not is_consensus:
                return
            
            winners_int = [int(w) if w not in ['X', -1, '-1'] else -1 for w in winners]
            
            if self.robot_id in winners_int:
                slot_index = winners_int.index(self.robot_id)
                if self.my_slot != slot_index:
                    self.my_slot = slot_index
                    self.offset_x = float(self.slot_offsets_x[slot_index])
                    self.offset_y = float(self.slot_offsets_y[slot_index])
            else:
                self.my_slot = -1
        except Exception as e:
            pass

    def neighbor_callback(self, msg, neighbor_id):
        # 持續更新所有其他車輛的最新狀態
        self.neighbor_data[neighbor_id] = msg.data

    def update_control_loop(self):
        self.loop_counter += 1
        
        if self.my_slot == -1:
            self.broadcast_state()
            return

        perceived_x = self.state[0]
        perceived_y = self.state[1]

        if self.sensor_quality == 'Poor':
            self.accumulated_error_x += random.gauss(0.0, 0.002)
            self.accumulated_error_y += random.gauss(0.0, 0.002)
            perceived_x += self.accumulated_error_x
            perceived_y += self.accumulated_error_y

        u_x = -self.k_goal * (perceived_x - self.offset_x)
        u_y = -self.k_goal * (perceived_y - self.offset_y)
        
        # ==========================================
        # ★ 修改 3：動態拓撲運算 (K-Nearest Neighbors)
        # ==========================================
        # 1. 計算自己與所有已知車輛的距離
        distance_list = []
        for j, data_j in self.neighbor_data.items():
            x_j = float(data_j[0])
            y_j = float(data_j[1])
            dist = math.hypot(perceived_x - x_j, perceived_y - y_j)
            distance_list.append((dist, j, data_j))
            
        # 2. 依照距離由近到遠排序，並只取前 k_nearest 台車
        distance_list.sort(key=lambda item: item[0])
        active_neighbors = distance_list[:self.k_nearest]
        
        # 3. 只對這 k 台最近的車產生共識引力
        for dist, j, data_j in active_neighbors:
            x_j = float(data_j[0])
            y_j = float(data_j[1])
            offset_x_j = float(data_j[2])
            offset_y_j = float(data_j[3])
            
            u_x -= self.k_consensus * ((perceived_x - x_j) - (self.offset_x - offset_x_j))
            u_y -= self.k_consensus * ((perceived_y - y_j) - (self.offset_y - offset_y_j))
            
        speed_mag = math.hypot(u_x, u_y)
        if speed_mag > self.max_speed and speed_mag > 0.0:
            scale = self.max_speed / speed_mag
            u_x *= scale
            u_y *= scale

        self.state[0] += u_x * self.timer_period
        self.state[1] += u_y * self.timer_period
        
        self.broadcast_state()
            
    def broadcast_state(self):
        msg = Float32MultiArray()
        msg.data = [float(self.state[0]), float(self.state[1]), float(self.offset_x), float(self.offset_y)]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FormationControlAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()