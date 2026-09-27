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

# 修改後的寫法：
from pgmpy.models import BayesianNetwork as DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# ==========================================
# 貝氏網路共用設定與模型建立 (與小車大腦完全一致)
# ==========================================
TIME_BIN_SIZE = 0.5
TIME_MAX_LIMIT = 10.0
NUM_TIME_BINS = int(TIME_MAX_LIMIT / TIME_BIN_SIZE)

BATTERY_BIN_SIZE = 0.1
BATTERY_MIN_LIMIT =10.9
BATTERY_MAX_LIMIT =12.1
NUM_BATTERY_BINS = int((BATTERY_MAX_LIMIT-BATTERY_MIN_LIMIT)/ BATTERY_BIN_SIZE)

def get_time_states_list():
    states = [f"{i*TIME_BIN_SIZE:.1f}~{(i+1)*TIME_BIN_SIZE:.1f}s" for i in range(NUM_TIME_BINS)]
    states.append(f"{TIME_MAX_LIMIT:.1f}~s")
    return states

def get_battery_states_list():
    # 修正：使用 BATTERY_BIN_SIZE 並將單位修正為 V
    states = [f"{BATTERY_MIN_LIMIT + i * BATTERY_BIN_SIZE:.1f}~{BATTERY_MIN_LIMIT + (i + 1) * BATTERY_BIN_SIZE:.1f}V" 
              for i in range(NUM_BATTERY_BINS)]
    states.append(f"{BATTERY_MAX_LIMIT:.1f}~V")
    return states

def battery_state(voltage: float) -> str:
    states = get_battery_states_list()
    if voltage <= BATTERY_MIN_LIMIT: 
        return states[0]
    if voltage >= BATTERY_MAX_LIMIT:
        return states[-1]
    # 修正：正規化減去最小值，再除以電量間隔
    idx = int((voltage - BATTERY_MIN_LIMIT) / BATTERY_BIN_SIZE)
    idx = max(0, min(idx, len(states) - 1))
    return states[idx]

def get_time_state(time_required: float) -> str:
    if time_required >= TIME_MAX_LIMIT: 
        return f"{TIME_MAX_LIMIT:.1f}~s"
    idx = int(time_required / TIME_BIN_SIZE)
    if idx < 0: idx = 0
    return get_time_states_list()[idx]

def build_bayesian_network():
    # 1. 補上 ('C', 'B')，使圖結構與 cpd_B 的雙條件對齊
    model = DiscreteBayesianNetwork([
        ('A', 'B'),
        ('C', 'B'),
        ('B', 'G'),
        ('C', 'F'),
        ('D', 'F'),
        ('F', 'G'),
    ])

    # 2. 時間與電量狀態
    time_states_list = get_time_states_list()
    num_time_states = len(time_states_list)

    battery_states_list = get_battery_states_list()
    num_battery_states = len(battery_states_list)

    # 3. 節點 C、D 先驗分佈
    cpd_C = TabularCPD(
        'C',
        num_time_states,
        [[1.0 / num_time_states]] * num_time_states,
        state_names={'C': time_states_list},
    )

    cpd_D = TabularCPD(
        'D',
        num_battery_states,
        [[1.0 / num_battery_states]] * num_battery_states,
        state_names={'D': battery_states_list},
    )

    # 4. 節點 F 條件機率
    denom = max(num_time_states - 1, 1)
    yes_table_F = []
    for i in range(num_time_states):
        ratio = i / denom
        start_p = max(0.005, 0.900 - 0.88 * ratio)
        end_p = max(0.010, 0.999 - 0.88 * ratio)
        row = np.linspace(start_p, end_p, num_battery_states)
        yes_table_F.append(row)

    yes_F = [p for row in yes_table_F for p in row]
    no_F = [round(1.0 - p, 4) for p in yes_F]

    cpd_F = TabularCPD(
        'F',
        2,
        [yes_F, no_F],
        evidence=['C', 'D'],
        evidence_card=[num_time_states, num_battery_states],
        state_names={
            'F': ['Yes', 'No'],
            'C': cpd_C.state_names['C'],
            'D': cpd_D.state_names['D'],
        },
    )

    # 5. 節點 A 先驗分佈
    cpd_A = TabularCPD(
        'A', 2, [[0.5], [0.5]], state_names={'A': ['Good', 'Poor']}
    )

    # 6. 節點 B 條件機率（對齊 A 與 C）
    yes_B_low, no_B_high = [], []
    for a_state in ['Good', 'Poor']:
        for i in range(num_time_states):
            b_high = (
                0.01 + (0.01 * i) if a_state == 'Good' else 0.10 + (0.05 * i)
            )
            b_high = min(0.999, b_high)
            no_B_high.append(round(b_high, 4))
            yes_B_low.append(round(1.0 - b_high, 4))

    cpd_B = TabularCPD(
        'B',
        2,
        [yes_B_low, no_B_high],
        evidence=['A', 'C'],  # 修正：父節點為 A 與 C
        evidence_card=[2, num_time_states],
        state_names={
            'B': ['Low', 'High'],  # 修正：自身鍵名為 B
            'A': cpd_A.state_names['A'],  # 修正：鍵名對應父節點 A
            'C': cpd_C.state_names['C'],  # 修正：鍵名對應父節點 C
        },
    )

    # 7. 節點 G 條件機率（狀態名稱與 cpd_B 一致）
    cpd_G = TabularCPD(
        'G',
        2,
        [[0.999, 0.010, 0.010, 0.001], [0.001, 0.990, 0.990, 0.999]],
        evidence=['B', 'F'],
        evidence_card=[2, 2],
        state_names={
            'G': ['Yes', 'No'],
            'B': ['Low', 'High'],  # 修正：必須與 cpd_B 定義的狀態一致
            'F': ['Yes', 'No'],
        },
    )

    model.add_cpds(cpd_A, cpd_B, cpd_C, cpd_D, cpd_F, cpd_G)
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

        self.voltage_rate = 0.050
        self.robots_state = {} # 儲存所有小車的最新狀態
        
        # ★ 新增：記憶上一次的分配結果 (格式: {robot_id: slot_index})
        self.current_assignment = {}
        self.get_logger().info("指揮官正在啟動貝氏網路模型...")
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        self.prob_cache = {}

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
                
                # 1. 建立快取鍵值 (以離散狀態字串作為 Tuple Key)
                state_c = get_time_state(time_req)
                state_d = battery_state(exp_battery)
                state_a = robot['sensor_quality']
                cache_key = (state_c, state_d, state_a)

                # 2. 檢查快取
                if cache_key in self.prob_cache:
                    success_prob = self.prob_cache[cache_key]
                else:
                    try:
                        result = self.infer.query(
                            variables=['G'],
                            evidence={'C': state_c, 'D': state_d, 'A': state_a}
                        )
                        success_prob = float(result.values[0])
                    except Exception:
                        success_prob = 0.0
                    
                    # 存入快取供後續車輛或下一秒直接複用
                    self.prob_cache[cache_key] = success_prob

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