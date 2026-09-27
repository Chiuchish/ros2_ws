import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String
import json
import math
import time
import warnings
import numpy as np
from scipy.optimize import linear_sum_assignment

warnings.filterwarnings("ignore", category=FutureWarning)

from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# ==========================================
# 貝氏網路共用設定與模型建立 (與小車大腦完全一致)
# ==========================================
TIME_BIN_SIZE = 0.5
TIME_MAX_LIMIT = 10.0
NUM_TIME_BINS = int(TIME_MAX_LIMIT / TIME_BIN_SIZE)

def get_time_states_list():
    states = [f"{i*TIME_BIN_SIZE:.1f}~{(i+1)*TIME_BIN_SIZE:.1f}s" for i in range(NUM_TIME_BINS)]
    states.append(f"{TIME_MAX_LIMIT:.1f}~s")
    return states

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
    return get_time_states_list()[idx]

def build_bayesian_network():
    model = DiscreteBayesianNetwork([
        ('A', 'E'), ('B', 'F'),
        ('E', 'K'), ('F', 'K'),
        ('I', 'L'), ('J', 'L'),
        ('Q', 'P'), ('I', 'P'),    
        ('K', 'M'), ('L', 'M'), ('P', 'M') 
    ])

    cpd_A = TabularCPD('A', 2, [[0.999], [0.001]], state_names={'A': ['Yes', 'No']})
    cpd_B = TabularCPD('B', 2, [[0.999], [0.001]], state_names={'B': ['Yes', 'No']})
    
    time_states_list = get_time_states_list()
    num_time_states = len(time_states_list)
    cpd_I = TabularCPD('I', num_time_states, [[1.0 / num_time_states]] * num_time_states, state_names={'I': time_states_list})
    
    cpd_J = TabularCPD('J', 5, [[0.200]]*5, state_names={'J': ['11.2~10.9V','11.5~11.2V','11.8~11.5V','12.1~11.8V','12.4~12.1V']})
    cpd_E = TabularCPD('E', 2, [[0.9, 0.01 ], [0.1, 0.99 ]], evidence=['A'], evidence_card=[2], state_names={'E': ['Good','Bad'], 'A': ['Yes','No']})
    cpd_F = TabularCPD('F', 2, [[0.9, 0.001], [0.1, 0.999]], evidence=['B'], evidence_card=[2], state_names={'F': ['Good','Bad'], 'B': ['Yes','No']})

    yes_K = [0.999, 0.005, 0.005, 0.003]
    no_K = [1-p for p in yes_K]
    cpd_K = TabularCPD('K', 2, [yes_K, no_K], evidence=['E','F'], evidence_card=[2,2], state_names={'K':['Yes','No'],'E':['Good','Bad'],'F':['Good','Bad']})

    W_TIME, W_BATTERY = 0.70, 0.30   
    yes_table_L = []
    for i in range(num_time_states):       
        row = []
        for j in range(5):
            time_score = (1.0 - (i / float(num_time_states - 1))) ** 1.5
            battery_score = j / 4.0
            prob = 0.001 + 0.998 * ((W_TIME * time_score) + (W_BATTERY * battery_score))
            row.append(round(prob, 4))
        yes_table_L.append(row)

    no_L  = [1-p for row in yes_table_L for p in row]
    yes_L = [p for row in yes_table_L for p in row]
    cpd_L = TabularCPD('L', 2, [yes_L, no_L], evidence=['I','J'], evidence_card=[num_time_states, 5], state_names={'L':['Yes','No'], 'I':cpd_I.state_names['I'], 'J':cpd_J.state_names['J']})

    cpd_Q = TabularCPD('Q', 2, [[0.5], [0.5]], state_names={'Q': ['Good', 'Poor']})

    yes_P_low, no_P_high = [], []
    for q_state in ['Good', 'Poor']:
        for i in range(num_time_states):
            p_high = 0.01 + (0.01 * i) if q_state == 'Good' else 0.10 + (0.05 * i)
            p_high = min(0.999, p_high)
            no_P_high.append(round(p_high, 4))
            yes_P_low.append(round(1.0 - p_high, 4))

    cpd_P = TabularCPD('P', 2, [yes_P_low, no_P_high], evidence=['Q', 'I'], evidence_card=[2, num_time_states], state_names={'P': ['Low', 'High'], 'Q': ['Good', 'Poor'], 'I': cpd_I.state_names['I']})

    yes_M = [0.999, 0.600, 0.010, 0.001, 0.010, 0.001, 0.001, 0.001]
    no_M  = [1 - p for p in yes_M]
    cpd_M = TabularCPD('M', 2, [yes_M, no_M], evidence=['K', 'L', 'P'], evidence_card=[2, 2, 2], state_names={'M': ['Yes', 'No'], 'K': ['Yes', 'No'], 'L': ['Yes', 'No'], 'P': ['Low', 'High']})

    model.add_cpds(cpd_A, cpd_B, cpd_I, cpd_J, cpd_E, cpd_F, cpd_K, cpd_L, cpd_Q, cpd_P, cpd_M)
    assert model.check_model()
    return model

# ==========================================
# 中心式指揮官節點 (Central Dispatcher)
# ==========================================
class CentralDispatcherNode(Node):
    def __init__(self):
        super().__init__('central_dispatcher')
        
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('num_slots', 4)
        self.declare_parameter('target_center_x', 5.0)
        self.declare_parameter('target_center_y', 5.0)
        
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('slot_offsets_x', value=[0.0, -1.0, -1.0, -2.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0, 1.0, -1.0, 0.0], descriptor=array_desc)

        self.num_robots = self.get_parameter('num_robots').value
        self.num_slots = self.get_parameter('num_slots').value
        self.target_center_x = self.get_parameter('target_center_x').value
        self.target_center_y = self.get_parameter('target_center_y').value
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value or [0.0, -1.0, -1.0, -2.0])
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value or [0.0, 1.0, -1.0, 0.0])

        self.voltage_rate = 0.0050
        self.robots_state = {} # 儲存所有小車的最新狀態
        
        # ★ 新增：記憶上一次的分配結果 (格式: {robot_id: slot_index})
        self.current_assignment = {}
        self.get_logger().info("指揮官正在啟動貝氏網路模型...")
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        # 訂閱遙測資料與發布分配命令
        self.create_subscription(String, '/central_telemetry', self.telemetry_callback, 10)
        self.assignment_pub = self.create_publisher(String, '/central_assignment', 10)
        
        # 週期性檢查與計算 (1Hz)
        self.create_timer(1.0, self.optimize_and_dispatch)
        self.get_logger().info(f"指揮官就緒！等待 {self.num_robots} 台小車上線...")

    def telemetry_callback(self, msg):
        try:
            info = json.loads(msg.data)
            r_id = info['id']
            self.robots_state[r_id] = info
        except Exception as e:
            self.get_logger().error(f"解析遙測資料失敗: {e}")

    def optimize_and_dispatch(self):
        # 必須確保所有小車都已經上線回報
        if len(self.robots_state) < self.num_robots:
            return

        calc_start = time.time()
        robot_ids = sorted(list(self.robots_state.keys())) 
        
        # ==========================================
        # 1. 建立「純物理」勝率矩陣 (Base Utility Matrix)
        # ==========================================
        base_utility_matrix = np.zeros((self.num_robots, self.num_slots))

        for i, r_id in enumerate(robot_ids):
            robot = self.robots_state[r_id]
            for j in range(self.num_slots):
                abs_slot_x = self.target_center_x + self.slot_offsets_x[j]
                abs_slot_y = self.target_center_y + self.slot_offsets_y[j]
                
                distance = math.hypot(robot['pos_x'] - abs_slot_x, robot['pos_y'] - abs_slot_y)
                time_req = (distance / robot['max_speed']) if robot['max_speed'] > 0 else 99.0
                exp_battery = robot['voltage'] - (self.voltage_rate * time_req)
                
                try:
                    result = self.infer.query(
                        variables=['M'],
                        evidence={
                            'I': get_time_state(time_req),
                            'J': battery_state(exp_battery),
                            'Q': robot['sensor_quality']
                        }
                    )
                    success_prob = result.values[0] 
                except:
                    success_prob = 0.0

                tie_breaker = max(0.0, (TIME_MAX_LIMIT - time_req)) * 0.0001
                base_utility_matrix[i][j] = success_prob + tie_breaker

        # ==========================================
        # 2. 複製矩陣並疊加「遲滯加分」(Hysteresis Matrix)
        # ==========================================
        utility_matrix = np.copy(base_utility_matrix)
        for i, r_id in enumerate(robot_ids):
            if r_id in self.current_assignment:
                prev_slot = self.current_assignment[r_id]
                # 0.3 代表 30% 勝率絕對優勢，強制鎖定現狀避免震顫
                utility_matrix[i][prev_slot] += 0.3 

        # ==========================================
        # 3. 匈牙利演算法最佳化
        # ==========================================
        # 因為加了 0.3 後 utility 可能大於 1.0，用 2.0 去減確保 cost 為正數
        cost_matrix = 2.0 - utility_matrix 
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # ==========================================
        # 4. 寫入新記憶與計算真實總分
        # ==========================================
        winners = [-1] * self.num_slots
        total_base_utility = 0.0
        
        # 建立全新的記憶字典，避免舊資料殘留
        new_assignment = {} 
        
        for r_idx, s_idx in zip(row_ind, col_ind):
            actual_robot_id = robot_ids[r_idx]
            slot_idx = int(s_idx) # ★ 關鍵：強制轉回標準 Python int，消除 NumPy 型別
            
            winners[slot_idx] = actual_robot_id
            new_assignment[actual_robot_id] = slot_idx
            
            # 從「純物理矩陣」中取值，這樣終端機印出來的數字才會是真實的 100% 準確！
            total_base_utility += base_utility_matrix[r_idx][slot_idx]

        # 覆蓋指揮官的記憶
        self.current_assignment = new_assignment

        # 發布最終名單
        msg = String()
        msg.data = json.dumps({'winners': winners})
        self.assignment_pub.publish(msg)

        elapsed = time.time() - calc_start
        self.get_logger().info(f"🏆 匈牙利演算法完成 (耗時 {elapsed:.3f}s)")
        self.get_logger().info(f"📍 最佳分配名單: {winners} | 全局物理總勝率: {total_base_utility:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = CentralDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()