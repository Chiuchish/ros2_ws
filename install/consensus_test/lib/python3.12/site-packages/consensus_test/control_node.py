import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import json
import traceback

class FormationControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        
        # ==========================================
        # 1. 宣告與讀取 Launch 傳入的參數
        # ==========================================
        self.declare_parameter('robot_id', 0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        int_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        
        self.declare_parameter('neighbors', descriptor=int_array_desc)
        self.declare_parameter('initial_value', descriptor=array_desc) 
        
        self.declare_parameter('slot_offsets_x', descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', descriptor=array_desc)

        # 強制轉型：確保 robot_id 絕對是 int
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.neighbors = self.get_parameter('neighbors').value or []
        # 強制轉換為 list
        self.state = list(self.get_parameter('initial_value').value or [0.0, 0.0])
        
        # 讀取偏移量，若讀取失敗則給予防呆的預設陣型 (避免 IndexError)
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])
        
        # 內部狀態記憶體
        self.my_slot = -1  # 目前得標的位置 (-1 代表未分配)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.neighbor_data = {} 
        self.loop_counter = 0 
        
        # ==========================================
        # 2. 【核心橋梁】訂閱自己「大腦」的拍賣結果
        # ==========================================
        self.create_subscription(
            String, 
            f'/robot_{self.robot_id}/auction_state', 
            self.auction_callback, 
            10
        )

        # ==========================================
        # 3. P2P 運動狀態發布與訂閱
        # ==========================================
        pub_topic = f'/robot_{self.robot_id}/kinematic_state'
        self.publisher_ = self.create_publisher(Float32MultiArray, pub_topic, 10)
        
        for j in self.neighbors:
            sub_topic = f'/robot_{j}/kinematic_state'
            self.create_subscription(
                Float32MultiArray, 
                sub_topic, 
                lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 
                10
            )
            
        # 4. 控制迴圈 (50Hz)
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.update_control_loop)
        self.k = 0.5  # 共識引力增益

    def auction_callback(self, msg):
        """接收拍賣結果，查表得出自己的目標偏移量 (δ)"""
        try:
            data = json.loads(msg.data)
            winners = data['winners']
            
            # 新增：讀取大腦傳來的共識訊號 (預設為 False 以防舊版節點)
            is_consensus = data.get('is_consensus', False)
            
            if not is_consensus:
                # 拍賣還在激烈進行中，忽略這些過渡期的結果，繼續原地待命
                return
            
            # 強制將 winners 轉為整數比對 (處理潛在型別問題)
            winners_int = [int(w) if w not in ['X', -1, '-1'] else -1 for w in winners]
            
            if self.robot_id in winners_int:
                slot_index = winners_int.index(self.robot_id)
                # 如果被分配到新的位置，就更新控制目標
                if self.my_slot != slot_index:
                    self.my_slot = slot_index
                    self.offset_x = float(self.slot_offsets_x[slot_index])
                    self.offset_y = float(self.slot_offsets_y[slot_index])
                    self.get_logger().info(
                        f"收到指令：前往 Slot {self.my_slot} (目標偏移量: X={self.offset_x}, Y={self.offset_y})"
                    )
            else:
                if self.my_slot != -1:
                    self.get_logger().warn("⚠️ 失去原本的 Slot，退回原地待命！")
                self.my_slot = -1
        except Exception as e:
            # 拒絕無聲崩潰！將錯誤詳細印出
            self.get_logger().error(f"❌ 解析大腦訊息失敗: {e}\n{traceback.format_exc()}")

    def neighbor_callback(self, msg, neighbor_id):
        """收到鄰居物理狀態"""
        self.neighbor_data[neighbor_id] = msg.data

    def update_control_loop(self):
        """核心編隊控制演算法"""
        self.loop_counter += 1
        
        # 如果大腦還沒分配位置，原地待命
        if self.my_slot == -1:
            self.broadcast_state()
            # 即使在待命，也每秒印出一次狀態，證明節點還活著
            if self.loop_counter % 50 == 0:
                self.get_logger().info("⏳ 尚未分配到 Slot，原地待命中...")
            return

        u_x = 0.0
        u_y = 0.0
        
        for j, data_j in self.neighbor_data.items():
            x_j = float(data_j[0])
            y_j = float(data_j[1])
            offset_x_j = float(data_j[2])
            offset_y_j = float(data_j[3])
            
            # 二階編隊共識公式：加上偏移量的誤差補償
            u_x -= self.k * ((self.state[0] - x_j) - (self.offset_x - offset_x_j))
            u_y -= self.k * ((self.state[1] - y_j) - (self.offset_y - offset_y_j))
            
        # 歐拉積分更新座標
        self.state[0] += u_x * self.timer_period
        self.state[1] += u_y * self.timer_period
        
        self.broadcast_state()
        
        # 降頻列印 (每秒才印一次)
        if self.loop_counter % 50 == 0:
            self.get_logger().info(f'📍 座標: ({self.state[0]:.2f}, {self.state[1]:.2f}) | 控制力: ({u_x:.2f}, {u_y:.2f})')
            
    def broadcast_state(self):
        """廣播自己的絕對座標與相對目標偏移量"""
        msg = Float32MultiArray()
        # 格式: [本體X, 本體Y, 目標偏移X, 目標偏移Y]
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