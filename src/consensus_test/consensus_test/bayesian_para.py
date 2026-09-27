import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String, Bool
import json
import random
import math
import time
import warnings

# 忽略 pgmpy 內部升級所產生的 FutureWarning，保持終端機畫面乾淨
warnings.filterwarnings("ignore", category=FutureWarning)

from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# ==========================================
# 動態時間區間設定 (提高解析度)
# ==========================================
TIME_BIN_SIZE = 0.2      # 每個區間 0.5 秒
TIME_MAX_LIMIT = 15.0    # 最大考量時間 10.0 秒
NUM_TIME_BINS = int(TIME_MAX_LIMIT / TIME_BIN_SIZE)

def get_time_states_list():
    states = [f"{i*TIME_BIN_SIZE:.1f}~{(i+1)*TIME_BIN_SIZE:.1f}s" for i in range(NUM_TIME_BINS)]
    states.append(f"{TIME_MAX_LIMIT:.1f}~s")
    return states

# ==========================================
# 輔助函式 (狀態區間分類)
# ==========================================
def battery_state(voltage):
    if voltage >= 12.1: return '12.4~12.1V'
    elif voltage >= 11.8: return '12.1~11.8V'
    elif voltage >= 11.5: return '11.8~11.5V'
    elif voltage >= 11.2: return '11.5~11.2V'
    else: return '11.2~10.9V'

def get_time_state(time_required: float) -> str:
    if time_required >= TIME_MAX_LIMIT: 
        return f"{TIME_MAX_LIMIT:.1f}~s"
        
    idx = int(time_required / TIME_BIN_SIZE)
    if idx < 0: idx = 0
    
    states = get_time_states_list()
    return states[idx]

def build_bayesian_network():
    """建立精簡版的離散貝氏網路，並動態生成 CPT 以利調整"""
    model = DiscreteBayesianNetwork([
        ('A', 'E'), ('B', 'F'),
        ('E', 'K'), ('F', 'K'),
        ('I', 'L'), ('J', 'L'),
        ('K', 'M'), ('L', 'M')
    ])

    cpd_A = TabularCPD('A', 2, [[0.999], [0.001]], state_names={'A': ['Yes', 'No']})
    cpd_B = TabularCPD('B', 2, [[0.999], [0.001]], state_names={'B': ['Yes', 'No']})
    
    time_states_list = get_time_states_list()
    num_time_states = len(time_states_list)
    
    cpd_I = TabularCPD('I', num_time_states, [[1.0 / num_time_states]] * num_time_states,
                       state_names={'I': time_states_list})
    
    cpd_J = TabularCPD('J', 5, [[0.200]]*5,
                       state_names={'J': [
                           '11.2~10.9V','11.5~11.2V','11.8~11.5V','12.1~11.8V','12.4~12.1V'
                       ]})

    cpd_E = TabularCPD('E', 2, [[0.9, 0.01 ], [0.1, 0.99 ]],
                       evidence=['A'], evidence_card=[2], state_names={'E': ['Good','Bad'], 'A': ['Yes','No']})
    cpd_F = TabularCPD('F', 2, [[0.9, 0.001], [0.1, 0.999]],
                       evidence=['B'], evidence_card=[2], state_names={'F': ['Good','Bad'], 'B': ['Yes','No']})

    yes_K = [0.999, 0.005, 0.005, 0.003]
    no_K = [1-p for p in yes_K]
    cpd_K = TabularCPD('K', 2, [yes_K, no_K],
                       evidence=['E','F'], evidence_card=[2,2],
                       state_names={'K':['Yes','No'],'E':['Good','Bad'],'F':['Good','Bad']})

    # =========================================================================
    # ★ 動態生成 CPT 表
    # =========================================================================
    W_TIME = 0.80      
    W_BATTERY = 0.20   
    
    yes_table_L = []
    for i in range(num_time_states):       
        row = []
        for j in range(5):
            time_score = 1.0 - (i / float(num_time_states - 1))
            battery_score = j / 4.0
            
            time_score = time_score ** 1.5 
            total_score = (W_TIME * time_score) + (W_BATTERY * battery_score)
            
            prob = 0.001 + 0.998 * total_score
            row.append(round(prob, 4))
            
        yes_table_L.append(row)

    no_L  = [1-p for row in yes_table_L for p in row]
    yes_L = [p for row in yes_table_L for p in row]
    
    cpd_L = TabularCPD('L', 2, [yes_L, no_L],
                       evidence=['I','J'], evidence_card=[num_time_states, 5],
                       state_names={'L':['Yes','No'], 'I':cpd_I.state_names['I'], 'J':cpd_J.state_names['J']})

    cpd_M = TabularCPD('M', 2, [[0.999,0.010,0.010,0.001], [0.001,0.990,0.990,0.999]],
                       evidence=['K','L'], evidence_card=[2,2],
                       state_names={'M':['Yes','No'],'K':['Yes','No'],'L':['Yes','No']})

    model.add_cpds(cpd_A, cpd_B, cpd_I, cpd_J, cpd_E, cpd_F, cpd_K, cpd_L, cpd_M)
    assert model.check_model(), "模型檢查失敗！"
    return model


class BayesianAuctionNode(Node):
    def __init__(self):
        super().__init__('bayesian_auction_agent')

        # 參數宣告與讀取
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_slots', 4)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('init_pos_x', 0.0)
        self.declare_parameter('init_pos_y', 0.0)
        
        # 全域目標中心點
        self.declare_parameter('target_center_x', 0.0)
        self.declare_parameter('target_center_y', 0.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        int_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        
        self.declare_parameter('neighbors', descriptor=int_array_desc)
        self.declare_parameter('slot_offsets_x', descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', descriptor=array_desc)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_slots = self.get_parameter('num_slots').value
        self.neighbors = self.get_parameter('neighbors').value or []
        
        self.max_speed = self.get_parameter('max_speed').value
        self.pos_x = self.get_parameter('init_pos_x').value
        self.pos_y = self.get_parameter('init_pos_y').value
        
        self.target_center_x = self.get_parameter('target_center_x').value
        self.target_center_y = self.get_parameter('target_center_y').value
        
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])

        self.current_voltage = random.uniform(11.5, 12.4)
        self.voltage_rate = 0.0050 

        self.get_logger().info("正在建立高解析度離散貝氏網路模型...")
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        self.my_utilities = self.calculate_discrete_bayesian_utilities()

        self.highest_bids = [0.0] * self.num_slots
        self.winners = [-1] * self.num_slots
        
        self.cycle_count = 0
        self.idle_cycles = 0           
        self.consensus_reached = False 
        self.changed_by_neighbor = False 

        pub_topic = f'/robot_{self.robot_id}/auction_state'
        self.publisher_ = self.create_publisher(String, pub_topic, 10)

        for j in self.neighbors:
            sub_topic = f'/robot_{j}/auction_state'
            self.create_subscription(
                String, sub_topic, 
                lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
            )
            
        self.create_subscription(Bool, f'/robot_{self.robot_id}/trigger_reallocation', self.trigger_callback, 10)

        self.timer = self.create_timer(0.2, self.auction_loop)
        self.get_logger().info(f"Robot {self.robot_id} 啟動！(極速: {self.max_speed} m/s, 初始電壓: {self.current_voltage:.2f}V)")

    def calculate_discrete_bayesian_utilities(self):
        utilities = []
        calc_start_time = time.time()
        
        for slot_idx in range(self.num_slots):
            # 取得絕對世界座標
            offset_x = self.slot_offsets_x[slot_idx]
            offset_y = self.slot_offsets_y[slot_idx]
            abs_slot_x = self.target_center_x + offset_x
            abs_slot_y = self.target_center_y + offset_y
            
            # 計算距離與預期消耗時間
            distance = math.hypot(self.pos_x - abs_slot_x, self.pos_y - abs_slot_y)
            time_required = (distance / self.max_speed) if self.max_speed > 0 else 99.0
            
            expected_battery = self.current_voltage - (self.voltage_rate * time_required)
            
            time_state_str = get_time_state(time_required)
            battery_state_str = battery_state(expected_battery)
            
            try:
                result = self.infer.query(
                    variables=['M'],
                    evidence={
                        'I': time_state_str,
                        'J': battery_state_str
                    }
                )
                success_prob = result.values[0] 
            except Exception as e:
                self.get_logger().error(f"BN 推論失敗: {e}")
                success_prob = 0.0

            # 【關鍵修改】加入連續時間微小懲罰 (Tie-Breaker)，徹底打破平手僵局
            tie_breaker = max(0.0, (TIME_MAX_LIMIT - time_required)) * 0.0001
            final_prob = success_prob + tie_breaker

            utilities.append(round(final_prob, 4)) # 保留小數點後四位

        elapsed = time.time() - calc_start_time
        self.get_logger().info(f"隊形中心點目標({self.target_center_x}, {self.target_center_y})後計算完成。出價: {utilities}")
        
        return utilities

    def run_local_auction(self):
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
        if msg.data:
            self.get_logger().warn(f"🚨 內部狀態改變！強制打破共識，請求重新分配！")
            self.consensus_reached = False
            self.idle_cycles = 0
            self.changed_by_neighbor = True 

    def auction_loop(self):
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