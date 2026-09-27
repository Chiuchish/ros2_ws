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
TIME_BIN_SIZE = 0.1      # 每個區間 0.5 秒
TIME_MAX_LIMIT = 20.0    # 最大考量時間 10.0 秒
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
        ('Q', 'P'), ('I', 'P'),    # 設備品質與耗時 共同決定 定位誤差
        ('K', 'M'), ('L', 'M'), ('P', 'M') # 誤差現在也會直接影響最終成功率
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

    W_TIME = 0.7      
    W_BATTERY = 0.3   
    
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

    # 2. 定義節點 Q (設備品質，先驗機率設為 0.5，實際推論時會作為 Evidence 傳入)
    cpd_Q = TabularCPD('Q', 2, [[0.5], [0.5]], state_names={'Q': ['Good', 'Poor']})

    # 3. 動態生成節點 P (定位誤差: Low / High) 的 CPT
    # 依賴於 Q (2 種狀態) 與 I (num_time_states 種狀態)
    yes_P_low = []
    no_P_high = []
    
    # 必須按照 pgmpy 的扁平化規則：左側變數 (Q) 變換最慢，右側 (I) 變換最快
    for q_state in ['Good', 'Poor']:
        for i in range(num_time_states):
            if q_state == 'Good':
                # 高品質：基礎誤差 1%，每單位時間飄移 1%
                p_high = 0.01 + (0.01 * i)
            else:
                # 低品質：基礎誤差 10%，每單位時間飄移 5%
                p_high = 0.10 + (0.05 * i)
                
            p_high = min(0.999, p_high) # 避免機率超過 1.0
            no_P_high.append(round(p_high, 4))
            yes_P_low.append(round(1.0 - p_high, 4))

    cpd_P = TabularCPD('P', 2, [yes_P_low, no_P_high],
                       evidence=['Q', 'I'], evidence_card=[2, num_time_states],
                       state_names={'P': ['Low', 'High'], 'Q': ['Good', 'Poor'], 'I': cpd_I.state_names['I']})

    # 4. 更新節點 M (最終成功率)，現在依賴 K(硬體), L(適合度), P(誤差)
    # 順序組合: (Yes, Yes, Low), (Yes, Yes, High), (Yes, No, Low)... 等 8 種
    yes_M = [0.999, 0.600, 0.010, 0.001, 0.010, 0.001, 0.001, 0.001]
    no_M  = [1 - p for p in yes_M]
    cpd_M = TabularCPD('M', 2, [yes_M, no_M],
                       evidence=['K', 'L', 'P'], evidence_card=[2, 2, 2],
                       state_names={
                           'M': ['Yes', 'No'],
                           'K': ['Yes', 'No'],
                           'L': ['Yes', 'No'],
                           'P': ['Low', 'High']
                       })

    # 將 Q 和 P 加入模型中
    model.add_cpds(cpd_A, cpd_B, cpd_I, cpd_J, cpd_E, cpd_F, cpd_K, cpd_L, cpd_Q, cpd_P, cpd_M)
    assert model.check_model(), "模型檢查失敗！"
    return model


class BayesianAuctionNode(Node):
    def __init__(self):
        super().__init__('bayesian_auction_agent')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_slots', 4)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('init_pos_x', 0.0)
        self.declare_parameter('init_pos_y', 0.0)
        
        self.declare_parameter('target_center_x', 0.0)
        self.declare_parameter('target_center_y', 0.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        int_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        self.declare_parameter('neighbors', value=[0, 1], descriptor=int_array_desc)
        self.declare_parameter('slot_offsets_x', descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', descriptor=array_desc)
        self.declare_parameter('sensor_quality', 'Good')

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
        self.sensor_quality = self.get_parameter('sensor_quality').value

        self.current_voltage = random.uniform(11.5, 12.4)
        self.voltage_rate = 0.0050 

        self.get_logger().info("正在建立離散貝氏網路模型...")
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        self.my_utilities = self.calculate_discrete_bayesian_utilities()

        # ==========================================
        # ★ 導入 Bertsekas 拍賣變數
        # ==========================================
        self.prices = [0.0] * self.num_slots  # 市場價格，取代原本的 highest_bids
        self.winners = [-1] * self.num_slots  
        self.epsilon = 0.05                   # 最小加價幅度
        
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

        self.timer = self.create_timer(0.01, self.auction_loop) # 若要觀察競價過程，可調慢至 1.0
        self.get_logger().info(f"Robot {self.robot_id} 啟動！(極速: {self.max_speed} m/s)")

    def calculate_discrete_bayesian_utilities(self):
        utilities = []
        calc_start_time = time.time()
        
        for slot_idx in range(self.num_slots):
            offset_x = self.slot_offsets_x[slot_idx]
            offset_y = self.slot_offsets_y[slot_idx]
            abs_slot_x = self.target_center_x + offset_x
            abs_slot_y = self.target_center_y + offset_y
            
            distance = math.hypot(self.pos_x - abs_slot_x, self.pos_y - abs_slot_y)
            time_required = (distance / self.max_speed) if self.max_speed > 0 else 99.0
            
            expected_battery = self.current_voltage - (self.voltage_rate * time_required)
            
            time_state_str = get_time_state(time_required)
            battery_state_str = battery_state(expected_battery)
            
            try:
                result = self.infer.query(
                    variables=['M'],
                    evidence={'I': time_state_str, 
                              'J': battery_state_str,
                              'Q': self.sensor_quality
                              }
                )
                success_prob = result.values[0] 
            except Exception as e:
                self.get_logger().error(f"BN 推論失敗: {e}")
                success_prob = 0.0

            tie_breaker = max(0.0, (TIME_MAX_LIMIT - time_required)) * 0.0001
            final_prob = success_prob + tie_breaker

            utilities.append(round(final_prob, 4)) 

        elapsed = time.time() - calc_start_time
        self.get_logger().info(f"🤖 Robot {self.robot_id} 各 Slot 勝率估值: {utilities}")
        
        return utilities

    # ==========================================
    # ★ Bertsekas 本地出價演算法
    # ==========================================
    def run_local_auction(self):
        # 如果我已經是某個位置的得標者，就安靜不出價
        if self.robot_id in self.winners:
            return False
            
        # 1. 計算所有 Slot 的「淨值 (Net Value)」= 適合度 - 目前市場價格
        net_values = [self.my_utilities[j] - self.prices[j] for j in range(self.num_slots)]
        
        # 2. 找出第一名與第二名的 Slot
        best_slot = -1
        best_val = -float('inf')
        second_best_val = -float('inf')
        
        for j in range(self.num_slots):
            val = net_values[j]
            if val > best_val:
                second_best_val = best_val
                best_val = val
                best_slot = j
            elif val > second_best_val:
                second_best_val = val
                
        # 3. 如果沒有第二名，就設為 0 作為基準
        if self.num_slots <= 1:
            second_best_val = 0.0
            
        if best_slot != -1:
            # 4. Bertsekas 加價邏輯：新價格 = 原價格 + (第一名淨值 - 第二名淨值) + epsilon
            bid_increment = best_val - second_best_val + self.epsilon
            new_price = self.prices[best_slot] + bid_increment
            #強制作四捨五入，消除浮點數尾數誤差
            new_price = round(new_price, 4)
            
            self.prices[best_slot] = new_price
            self.winners[best_slot] = self.robot_id
            
            self.get_logger().info(f"💰 Robot {self.robot_id} 對 Slot {best_slot} 提新價格: {new_price:.4f} (淨值差: {best_val - second_best_val:.4f}) Price update: {self.prices}")
            return True
            
        return False

    def neighbor_callback(self, msg, neighbor_id):
        try:
            data = json.loads(msg.data)
            n_prices = data.get('prices', data.get('bids', []))
            n_winners = data['winners']
            
            changed = False
            for slot_idx in range(self.num_slots):
                # 確保收到的價格是乾淨的 4 位小數
                clean_n_price = round(n_prices[slot_idx], 4)
                
                # 情況 1：發現市場上有人出「更高」的價格
                if clean_n_price > self.prices[slot_idx] + 1e-5:
                    if self.winners[slot_idx] == self.robot_id and n_winners[slot_idx] != self.robot_id:
                        self.get_logger().warn(f"😱 我在 Slot {slot_idx} 被 Robot {n_winners[slot_idx]} 出高價擠掉，重新尋找位置！")
                        
                    self.prices[slot_idx] = clean_n_price
                    self.winners[slot_idx] = n_winners[slot_idx]
                    changed = True
                    
                # 情況 2：【關鍵修復】價格「平手」時的斷路器 (Tie-Breaker)
                elif abs(clean_n_price - self.prices[slot_idx]) <= 1e-5:
                    # 如果價格一模一樣，我們讓 Robot ID 數字較大的車勝出！
                    if n_winners[slot_idx] != -1 and n_winners[slot_idx] > self.winners[slot_idx]:
                        if self.winners[slot_idx] == self.robot_id:
                            self.get_logger().warn(f"⚠️ 價格平手！但我 (ID:{self.robot_id}) 禮讓給 ID:{n_winners[slot_idx]}，重新尋找位置！")
                            
                        self.prices[slot_idx] = clean_n_price
                        self.winners[slot_idx] = n_winners[slot_idx]
                        changed = True
                    
            if changed:
                self.changed_by_neighbor = True
                if self.consensus_reached:
                    self.consensus_reached = False
                    self.idle_cycles = 0
        except Exception as e:
            pass

    def trigger_callback(self, msg):
        if msg.data:
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
            self.get_logger().info(f"✅ 達成完美價格共識！發送最終分配結果。")

        state_dict = {
            'prices': self.prices,
            'winners': self.winners,
            'is_consensus': is_consensus
        }
        msg = String()
        msg.data = json.dumps(state_dict)
        self.publisher_.publish(msg)
        
        if not is_consensus:
            display_winners = [w if w != -1 else 'X' for w in self.winners]
            self.get_logger().info(f"[Cycle {self.cycle_count}] 暫時得標者: {display_winners}")

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