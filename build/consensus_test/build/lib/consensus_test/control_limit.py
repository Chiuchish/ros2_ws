import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import json
import traceback
import math

class FormationControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')
        
        # ==========================================
        # 1. 宣告與讀取 Launch 傳入的參數
        # ==========================================
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_speed', 1.0) # 讀取極速
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        int_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        
        self.declare_parameter('neighbors', descriptor=int_array_desc)
        self.declare_parameter('initial_value', descriptor=array_desc) 
        self.declare_parameter('slot_offsets_x', descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', descriptor=array_desc)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.neighbors = self.get_parameter('neighbors').value or []
        self.state = list(self.get_parameter('initial_value').value or [0.0, 0.0])
        
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])
        
        self.my_slot = -1  
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.neighbor_data = {} 
        self.loop_counter = 0 
        
        self.create_subscription(String, f'/robot_{self.robot_id}/auction_state', self.auction_callback, 10)

        pub_topic = f'/robot_{self.robot_id}/kinematic_state'
        self.publisher_ = self.create_publisher(Float32MultiArray, pub_topic, 10)
        
        for j in self.neighbors:
            sub_topic = f'/robot_{j}/kinematic_state'
            self.create_subscription(
                Float32MultiArray, sub_topic, 
                lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
            )
            
        # 控制迴圈 (50Hz)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_control_loop)
        
        # 【關鍵修改】分離兩種引力的增益係數
        self.k_consensus = 0.5  # 維持陣型的引力 (相對)
        self.k_goal = 1.0       # 前往絕對座標的引力 (絕對)

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
                    # 這裡讀取到的 offset 已經是被 Launch 檔加上 target_center 的「絕對座標」了
                    self.offset_x = float(self.slot_offsets_x[slot_index])
                    self.offset_y = float(self.slot_offsets_y[slot_index])
                    self.get_logger().info(
                        f"🎯 大腦指示：前往 Slot {self.my_slot} (全域目標座標: X={self.offset_x}, Y={self.offset_y})"
                    )
            else:
                if self.my_slot != -1:
                    self.get_logger().warn("⚠️ 失去原本的 Slot，退回原地待命！")
                self.my_slot = -1
        except Exception as e:
            self.get_logger().error(f"❌ 解析大腦訊息失敗: {e}\n{traceback.format_exc()}")

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbor_data[neighbor_id] = msg.data

    def update_control_loop(self):
        self.loop_counter += 1
        
        if self.my_slot == -1:
            self.broadcast_state()
            return

        # 1. 絕對導航引力：把自己往「專屬的絕對目標點」拉
        u_x = -self.k_goal * (self.state[0] - self.offset_x)
        u_y = -self.k_goal * (self.state[1] - self.offset_y)
        
        # 2. 相對共識引力：維持與鄰居的相對陣型
        for j, data_j in self.neighbor_data.items():
            x_j = float(data_j[0])
            y_j = float(data_j[1])
            offset_x_j = float(data_j[2])
            offset_y_j = float(data_j[3])
            
            u_x -= self.k_consensus * ((self.state[0] - x_j) - (self.offset_x - offset_x_j))
            u_y -= self.k_consensus * ((self.state[1] - y_j) - (self.offset_y - offset_y_j))
            
        # 3. 速度限制 (Velocity Clamping)
        speed_mag = math.hypot(u_x, u_y)
        if speed_mag > self.max_speed and speed_mag > 0.0:
            scale = self.max_speed / speed_mag
            u_x *= scale
            u_y *= scale
            
        # 4. 歐拉積分更新座標
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