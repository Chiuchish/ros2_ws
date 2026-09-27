import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String, Bool
import json
import random
import math
import time

# 引入 pgmpy 相關套件 (學長的離散貝氏網路)
from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# ==========================================
# 學長的輔助函式 (狀態區間分類)
# ==========================================
def battery_state(voltage):
    if voltage >= 12.1: return '12.4~12.1V'
    elif voltage >= 11.8: return '12.1~11.8V'
    elif voltage >= 11.5: return '11.8~11.5V'
    elif voltage >= 11.2: return '11.5~11.2V'
    else: return '11.2~10.9V'

def get_time_state(time_required: float) -> str:
    states = ['30~33 sec', '33~36 sec', '36~39 sec', '39~42 sec', '42~45 sec',
              '45~48 sec', '48~51 sec', '51~54 sec', '54~57 sec', '57~60 sec',
              '60~63 sec', '63~66 sec', '66~69 sec', '69~72 sec', '72~75 sec',
              '75~78 sec', '78~81 sec', '81~ sec']
    lower = 30.0
    for state in states:
        if state == "81~ sec":
            if time_required >= 81.0: return state
        else:
            upper = lower + 3.0
            if lower <= time_required < upper: return state
            lower = upper
    return states[-1]

def build_bayesian_network():
    """建立精簡版的離散貝氏網路 (移除 Size 與 Load 相關節點 C, D, G, H)"""
    model = DiscreteBayesianNetwork([
        ('A', 'E'), ('B', 'F'),
        ('E', 'K'), ('F', 'K'),
        ('I', 'L'), ('J', 'L'),
        ('K', 'M'), ('L', 'M')
    ])

    cpd_A = TabularCPD('A', 2, [[0.999], [0.001]], state_names={'A': ['Yes', 'No']})
    cpd_B = TabularCPD('B', 2, [[0.999], [0.001]], state_names={'B': ['Yes', 'No']})
    
    cpd_I = TabularCPD('I', 18, [[0.056]]*10+[[0.055]]*8,
                       state_names={'I': [
                           '30~33 sec','33~36 sec','36~39 sec','39~42 sec','42~45 sec',
                           '45~48 sec','48~51 sec','51~54 sec','54~57 sec','57~60 sec',
                           '60~63 sec','63~66 sec','66~69 sec','69~72 sec','72~75 sec',
                           '75~78 sec','78~81 sec','81~ sec'
                       ]})
    cpd_J = TabularCPD('J', 5, [[0.200]]*5,
                       state_names={'J': [
                           '11.2~10.9V','11.5~11.2V','11.8~11.5V','12.1~11.8V','12.4~12.1V'
                       ]})

    cpd_E = TabularCPD('E', 2, [[0.9, 0.01 ], [0.1, 0.99 ]],
                       evidence=['A'], evidence_card=[2], state_names={'E': ['Good','Bad'], 'A': ['Yes','No']})
    cpd_F = TabularCPD('F', 2, [[0.9, 0.001], [0.1, 0.999]],
                       evidence=['B'], evidence_card=[2], state_names={'F': ['Good','Bad'], 'B': ['Yes','No']})

    # 原本的 K 取決於 E,F,G,H。現在剔除了 G,H，我們擷取當 G,H 都是 Good 時的機率作為新的 K 的 CPD
    yes_K = [0.999, 0.005, 0.005, 0.003]
    no_K = [1-p for p in yes_K]
    cpd_K = TabularCPD('K', 2, [yes_K, no_K],
                       evidence=['E','F'], evidence_card=[2,2],
                       state_names={'K':['Yes','No'],'E':['Good','Bad'],'F':['Good','Bad']})

    # L 取決於 I, J (保持不變)
    yes_table_L = [
        [0.900,0.925,0.950,0.975,0.999], [0.850,0.875,0.900,0.925,0.950], [0.800,0.825,0.850,0.875,0.900],
        [0.750,0.775,0.800,0.825,0.850], [0.700,0.725,0.750,0.775,0.800], [0.650,0.675,0.700,0.725,0.750],
        [0.600,0.625,0.650,0.675,0.700], [0.550,0.575,0.600,0.625,0.650], [0.500,0.525,0.550,0.575,0.600],
        [0.450,0.475,0.500,0.525,0.550], [0.400,0.425,0.450,0.475,0.500], [0.350,0.375,0.400,0.425,0.450],
        [0.300,0.325,0.350,0.375,0.400], [0.250,0.275,0.300,0.325,0.350], [0.200,0.225,0.250,0.275,0.300],
        [0.100,0.175,0.200,0.225,0.250], [0.100,0.125,0.150,0.175,0.200], [0.001,0.025,0.050,0.075,0.100]
    ]
    yes_L = [p for row in yes_table_L for p in row]
    no_L  = [1-p for p in yes_L]
    cpd_L = TabularCPD('L', 2, [yes_L, no_L],
                       evidence=['I','J'], evidence_card=[18,5],
                       state_names={'L':['Yes','No'], 'I':cpd_I.state_names['I'], 'J':cpd_J.state_names['J']})

    # M 取決於 K, L (保持不變)
    cpd_M = TabularCPD('M', 2, [[0.999,0.010,0.010,0.001], [0.001,0.990,0.990,0.999]],
                       evidence=['K','L'], evidence_card=[2,2],
                       state_names={'M':['Yes','No'],'K':['Yes','No'],'L':['Yes','No']})

    # 將剩餘的 CPD 加入模型
    model.add_cpds(cpd_A, cpd_B, cpd_I, cpd_J, cpd_E, cpd_F, cpd_K, cpd_L, cpd_M)
    assert model.check_model(), "模型檢查失敗！"
    return model


class BayesianAuctionNode(Node):
    def __init__(self):
        super().__init__('bayesian_auction_agent')

        # ==========================================
        # 1. 參數宣告與讀取
        # ==========================================
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_slots', 4)
        
        # 物理能力與初始位置
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('init_pos_x', 0.0)
        self.declare_parameter('init_pos_y', 0.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        int_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        
        self.declare_parameter('neighbors', descriptor=int_array_desc)
        self.declare_parameter('slot_offsets_x', descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', descriptor=array_desc)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_slots = self.get_parameter('num_slots').value
        self.neighbors = self.get_parameter('neighbors').value or []
        
        # 讀取實體屬性
        self.max_speed = self.get_parameter('max_speed').value
        self.pos_x = self.get_parameter('init_pos_x').value
        self.pos_y = self.get_parameter('init_pos_y').value
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])

        # 模擬讀取內部電池電壓狀態 (10.9V ~ 12.4V)
        self.current_voltage = random.uniform(11.5, 12.4)
        self.voltage_rate = 0.000200 # 學長的電壓消耗率參數

        # ==========================================
        # 2. 建立 pgmpy 貝氏網路推論引擎
        # ==========================================
        self.get_logger().info("正在建立 pgmpy 精簡版離散貝氏網路模型...")
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        # 透過貝氏網路計算對每個 Slot 的適合度 (出價)
        self.my_utilities = self.calculate_discrete_bayesian_utilities()

        # ==========================================
        # 3. 內部記憶體與狀態控制
        # ==========================================
        self.highest_bids = [0.0] * self.num_slots
        self.winners = [-1] * self.num_slots
        
        self.cycle_count = 0
        self.idle_cycles = 0           
        self.consensus_reached = False 
        self.changed_by_neighbor = False 

        # ==========================================
        # 4. P2P 通訊設定
        # ==========================================
        pub_topic = f'/robot_{self.robot_id}/auction_state'
        self.publisher_ = self.create_publisher(String, pub_topic, 10)

        for j in self.neighbors:
            sub_topic = f'/robot_{j}/auction_state'
            self.create_subscription(
                String, sub_topic, 
                lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
            )
            
        self.create_subscription(Bool, f'/robot_{self.robot_id}/trigger_reallocation', self.trigger_callback, 10)

        self.timer = self.create_timer(0.02, self.auction_loop)
        self.get_logger().info(f"Robot {self.robot_id} 啟動！(極速: {self.max_speed} m/s, 初始電壓: {self.current_voltage:.2f}V)")

    def calculate_discrete_bayesian_utilities(self):
        """
        利用精簡版 pgmpy 離散貝氏網路，計算前往各 Slot 的成功率
        """
        utilities = []
        calc_start_time = time.time()
        
        for slot_idx in range(self.num_slots):
            # 1. 空間與運動學計算 (距離 -> 預估耗時)
            slot_x = self.slot_offsets_x[slot_idx]
            slot_y = self.slot_offsets_y[slot_idx]
            distance = math.hypot(self.pos_x - slot_x, self.pos_y - slot_y)
            
            # 學長的公式：預估耗時 = (距離 / 速度) + 30.0 (緩衝時間)
            time_required = (distance / self.max_speed) + 30.0 if self.max_speed > 0 else 99.0
            
            # 預估到達該 Slot 時的剩餘電壓
            expected_battery = self.current_voltage - (self.voltage_rate * time_required)
            
            # 2. 將連續數值轉換為 BN 的離散區間標籤
            time_state_str = get_time_state(time_required)
            battery_state_str = battery_state(expected_battery)
            
            # 3. 進行貝氏網路推論 (移除 Size 與 Load 證據，專注於時間與電壓)
            try:
                result = self.infer.query(
                    variables=['M'],
                    evidence={
                        'I': time_state_str,
                        'J': battery_state_str
                    }
                )
                # 取得 M='Yes' (任務成功) 的機率值
                success_prob = result.values[0] # index 0 對應 'Yes'
            except Exception as e:
                self.get_logger().error(f"BN 推論失敗: {e}")
                success_prob = 0.0

            utilities.append(success_prob)

        # 為了觀察方便，保留三位小數
        utilities = [round(u, 3) for u in utilities]
        
        elapsed = time.time() - calc_start_time
        self.get_logger().info(f" BN 計算完成 (耗時 {elapsed:.2f}s)。各 Slot 出價: {utilities}")
        
        return utilities

    def run_local_auction(self):
        """本地出價，回傳是否發生了狀態改變"""
        best_slot = -1
        best_utility = -1.0
        local_changed = False
        
        for slot_idx in range(self.num_slots):
            my_bid = self.my_utilities[slot_idx]
            if my_bid > self.highest_bids[slot_idx] or self.winners[slot_idx] == self.robot_id:
                if my_bid > best_utility:
                    best_utility = my_bid
                    best_slot = slot_idx
                    
        if best_slot != -1:
            for slot_idx in range(self.num_slots):
                if slot_idx == best_slot:
                    if self.highest_bids[slot_idx] != self.my_utilities[slot_idx] or self.winners[slot_idx] != self.robot_id:
                        self.highest_bids[slot_idx] = self.my_utilities[slot_idx]
                        self.winners[slot_idx] = self.robot_id
                        local_changed = True
                elif self.winners[slot_idx] == self.robot_id:
                    self.highest_bids[slot_idx] = 0.0
                    self.winners[slot_idx] = -1
                    local_changed = True
                    
        return local_changed

    def neighbor_callback(self, msg, neighbor_id):
        """處理鄰居傳來的 P2P 資訊"""
        try:
            data = json.loads(msg.data)
            n_bids = data['bids']
            n_winners = data['winners']
            
            changed = False
            for slot_idx in range(self.num_slots):
                n_bid = n_bids[slot_idx]
                n_winner = n_winners[slot_idx]
                
                if n_bid > self.highest_bids[slot_idx] or (n_winner == -1 and self.winners[slot_idx] == n_winner):
                    self.highest_bids[slot_idx] = n_bid
                    self.winners[slot_idx] = n_winner
                    changed = True
                elif n_winner == self.winners[slot_idx] and n_bid != self.highest_bids[slot_idx]:
                    self.highest_bids[slot_idx] = n_bid
                    changed = True
                    
            if changed:
                self.changed_by_neighbor = True
                if self.consensus_reached:
                    self.consensus_reached = False
                    self.idle_cycles = 0
                    self.get_logger().info(f"⚠️ 收到 Robot {neighbor_id} 的【有效】新狀態，解除靜默，重啟協商！")
        except Exception as e:
            pass

    def trigger_callback(self, msg):
        """【喚醒機制】模擬內部貝氏網路狀態劇烈變化"""
        if msg.data:
            self.get_logger().warn(f"🚨 內部狀態改變！強制打破共識，請求重新分配！")
            self.consensus_reached = False
            self.idle_cycles = 0
            self.changed_by_neighbor = True 

    def auction_loop(self):
        """主決策迴圈"""
        if self.consensus_reached:
            return

        self.cycle_count += 1
        local_changed = self.run_local_auction()
        
        if not local_changed and not self.changed_by_neighbor:
            self.idle_cycles += 1
        else:
            self.idle_cycles = 0 
            
        self.changed_by_neighbor = False 
        
        is_consensus = False
        if self.idle_cycles >= 3 and -1 not in self.winners:
            self.consensus_reached = True
            is_consensus = True
            self.get_logger().info(f"✅ 達成完美共識！發送最終分配結果並進入靜默模式。")

        state_dict = {
            'bids': self.highest_bids,
            'winners': self.winners,
            'is_consensus': is_consensus
        }
        msg = String()
        msg.data = json.dumps(state_dict)
        self.publisher_.publish(msg)
        
        if not is_consensus:
            display_winners = [w if w != -1 else 'X' for w in self.winners]
            self.get_logger().info(f"[Cycle {self.cycle_count}] 得標者: {display_winners}")

def main(args=None):
    rclpy.init(args=args)
    node = BayesianAuctionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()