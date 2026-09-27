import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String, Float32MultiArray
import json
import math
import time
import warnings
import numpy as np
from scipy.optimize import linear_sum_assignment

warnings.filterwarnings("ignore", category=FutureWarning)

from pgmpy.models import BayesianNetwork as DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination

# ==========================================
# 貝氏網路共用配置與離散區間定義
# ==========================================
TIME_BIN_SIZE = 0.5
TIME_MAX_LIMIT = 10.0
NUM_TIME_BINS = int(TIME_MAX_LIMIT / TIME_BIN_SIZE)

BATTERY_BIN_SIZE = 0.1
BATTERY_MIN_LIMIT = 10.9
BATTERY_MAX_LIMIT = 12.1
NUM_BATTERY_BINS = int((BATTERY_MAX_LIMIT - BATTERY_MIN_LIMIT) / BATTERY_BIN_SIZE)

def get_time_states_list():
    states = [f"{i*TIME_BIN_SIZE:.1f}~{(i+1)*TIME_BIN_SIZE:.1f}s" for i in range(NUM_TIME_BINS)]
    states.append(f"{TIME_MAX_LIMIT:.1f}~s")
    return states

def get_battery_states_list():
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
    model = DiscreteBayesianNetwork([
        ('A', 'B'),
        ('C', 'B'),
        ('B', 'G'),
        ('C', 'F'),
        ('D', 'F'),
        ('F', 'G'),
    ])

    time_states_list = get_time_states_list()
    num_time_states = len(time_states_list)
    battery_states_list = get_battery_states_list()
    num_battery_states = len(battery_states_list)

    cpd_C = TabularCPD(
        'C', num_time_states, [[1.0 / num_time_states]] * num_time_states,
        state_names={'C': time_states_list}
    )
    cpd_D = TabularCPD(
        'D', num_battery_states, [[1.0 / num_battery_states]] * num_battery_states,
        state_names={'D': battery_states_list}
    )

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
        'F', 2, [yes_F, no_F],
        evidence=['C', 'D'],
        evidence_card=[num_time_states, num_battery_states],
        state_names={'F': ['Yes', 'No'], 'C': cpd_C.state_names['C'], 'D': cpd_D.state_names['D']}
    )

    cpd_A = TabularCPD('A', 2, [[0.5], [0.5]], state_names={'A': ['Good', 'Poor']})

    yes_B_low, no_B_high = [], []
    for a_state in ['Good', 'Poor']:
        for i in range(num_time_states):
            b_high = 0.01 + (0.01 * i) if a_state == 'Good' else 0.10 + (0.05 * i)
            b_high = min(0.999, b_high)
            no_B_high.append(round(b_high, 4))
            yes_B_low.append(round(1.0 - b_high, 4))

    cpd_B = TabularCPD(
        'B', 2, [yes_B_low, no_B_high],
        evidence=['A', 'C'],
        evidence_card=[2, num_time_states],
        state_names={'B': ['Low', 'High'], 'A': cpd_A.state_names['A'], 'C': cpd_C.state_names['C']}
    )

    cpd_G = TabularCPD(
        'G', 2,
        [[0.999, 0.010, 0.010, 0.001], [0.001, 0.990, 0.990, 0.999]],
        evidence=['B', 'F'],
        evidence_card=[2, 2],
        state_names={'G': ['Yes', 'No'], 'B': ['Low', 'High'], 'F': ['Yes', 'No']}
    )

    model.add_cpds(cpd_A, cpd_B, cpd_C, cpd_D, cpd_F, cpd_G)
    assert model.check_model()
    return model

# ==========================================
# 小車個體代理節點 (Robot Agent Node)
# ==========================================
class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent_node')

        # 1. 嚴格參數宣告 (ROS 2 Jazzy 規範)
        self.declare_parameter('robot_id', value=0)
        self.declare_parameter('num_robots', value=26)
        self.declare_parameter('num_slots', value=26)
        self.declare_parameter('max_speed', value=1.0)
        self.declare_parameter('sensor_quality', value='Good')
        self.declare_parameter('voltage', value=12.0)
        self.declare_parameter('pos_x', value=0.0)
        self.declare_parameter('pos_y', value=0.0)
        self.declare_parameter('target_center_x', value=0.0)
        self.declare_parameter('target_center_y', value=0.0)

        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('slot_offsets_x', value=[0.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0], descriptor=array_desc)

        # 讀取參數
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.num_slots = int(self.get_parameter('num_slots').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.sensor_quality = str(self.get_parameter('sensor_quality').value)
        self.voltage = float(self.get_parameter('voltage').value)
        self.pos_x = float(self.get_parameter('pos_x').value)
        self.pos_y = float(self.get_parameter('pos_y').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        self.target_center_y = float(self.get_parameter('target_center_y').value)
        self.slot_offsets_x = list(self.get_parameter('slot_offsets_x').value)
        self.slot_offsets_y = list(self.get_parameter('slot_offsets_y').value)

        self.voltage_rate = 0.050
        self.prob_cache = {}

        # 2. 建立本地貝氏大腦
        self.bayes_model = build_bayesian_network()
        self.infer = VariableElimination(self.bayes_model)

        # 3. 狀態與協商變數
        self.stage = 'ELECTION'  # 狀態機: ELECTION -> BIDDING -> ASSIGNED
        self.election_records = {}
        self.auctioneer_id = -1
        self.is_auctioneer = False
        self.collected_bids = {}
        self.assigned_slot = -1
        self.target_slot_pos = None

        # 4. 通訊話題 (全場去中心化 Topic)
        self.election_pub = self.create_publisher(String, '/auction_election', 10)
        self.create_subscription(String, '/auction_election', self.election_callback, 10)

        self.bid_pub = self.create_publisher(String, '/auction_bids', 10)
        self.create_subscription(String, '/auction_bids', self.bid_callback, 10)

        self.assignment_pub = self.create_publisher(String, '/formation_assignment', 10)
        self.create_subscription(String, '/formation_assignment', self.assignment_callback, 10)

        # 發布給 visualizer 與 evaluator 的狀態話題
        self.kinematic_pub = self.create_publisher(
            Float32MultiArray, f'/robot_{self.robot_id}/kinematic_state', 10
        )

        # 5. 計算自身基準能力並啟動選舉計時器
        self.capability_score = self.evaluate_capability()
        self.get_logger().info(
            f"🚗 Robot {self.robot_id} 初始化完成 | 感測器: {self.sensor_quality}, "
            f"電壓: {self.voltage:.2f}V, BN能力評分: {self.capability_score:.4f}"
        )

        # 定時器：廣播選舉資訊 (5Hz)
        self.election_timer = self.create_timer(0.2, self.broadcast_election)

    # ----------------------------------------------------
    # 貝氏網路推論工具函式
    # ----------------------------------------------------
    def evaluate_capability(self) -> float:
        """評估無距離干擾下的純硬體/電量基準可靠度"""
        c_min = get_time_states_list()[0]
        d_init = battery_state(self.voltage)
        a_state = self.sensor_quality
        key = (c_min, d_init, a_state)
        if key in self.prob_cache:
            return self.prob_cache[key]
        try:
            res = self.infer.query(variables=['G'], evidence={'C': c_min, 'D': d_init, 'A': a_state})
            score = float(res.values[0])
        except Exception:
            score = 0.0
        self.prob_cache[key] = score
        return score

    def calculate_slot_utility(self, slot_idx: int) -> float:
        """計算自己前往特定 Slot 的 BN 勝率"""
        target_x = self.target_center_x + self.slot_offsets_x[slot_idx]
        target_y = self.target_center_y + self.slot_offsets_y[slot_idx]
        dist = math.hypot(self.pos_x - target_x, self.pos_y - target_y)

        time_req = (dist / self.max_speed) if self.max_speed > 0 else 99.0
        exp_battery = self.voltage - (self.voltage_rate * time_req)

        c_state = get_time_state(time_req)
        d_state = battery_state(exp_battery)
        a_state = self.sensor_quality
        key = (c_state, d_state, a_state)

        if key in self.prob_cache:
            p = self.prob_cache[key]
        else:
            try:
                res = self.infer.query(variables=['G'], evidence={'C': c_state, 'D': d_state, 'A': a_state})
                p = float(res.values[0])
            except Exception:
                p = 0.0
            self.prob_cache[key] = p

        tie_breaker = max(0.0, (TIME_MAX_LIMIT - time_req)) * 0.0001
        return p + tie_breaker

    # ----------------------------------------------------
    # 第一階段：動態選舉 Auctioneer
    # ----------------------------------------------------
    def broadcast_election(self):
        if self.stage != 'ELECTION':
            return
        msg = String()
        msg.data = json.dumps({
            'id': self.robot_id,
            'capability_score': self.capability_score,
            'voltage': self.voltage
        })
        self.election_pub.publish(msg)

    def election_callback(self, msg):
        if self.stage != 'ELECTION':
            return
        data = json.loads(msg.data)
        r_id = data['id']
        self.election_records[r_id] = (data['capability_score'], data['voltage'])

        # 當收集齊全所有小車票數時，進行決定
        if len(self.election_records) == self.num_robots:
            self.election_timer.cancel()
            
            # 選舉依據：BN分數最高者勝出；若同分則依電壓、ID嚴格排序
            best_id = max(
                self.election_records.keys(),
                key=lambda r: (self.election_records[r][0], self.election_records[r][1], r)
            )
            self.auctioneer_id = best_id
            self.is_auctioneer = (self.robot_id == self.auctioneer_id)
            self.stage = 'BIDDING'

            if self.is_auctioneer:
                self.get_logger().info(
                    f"👑 我當選為 Auctioneer！(BN 分數: {self.capability_score:.4f})，準備主持最佳化..."
                )
            else:
                self.get_logger().info(
                    f"🤖 Robot {self.auctioneer_id} 當選為 Auctioneer，我將發送出價向量。"
                )

            # 進入投標階段
            self.submit_bids()

    # ----------------------------------------------------
    # 第二階段：投標與匈牙利分配
    # ----------------------------------------------------
    def submit_bids(self):
        # 1. 本地計算 26 個 slot 的效用
        my_utilities = [self.calculate_slot_utility(j) for j in range(self.num_slots)]

        # 如果自己是 Auctioneer，先存入自己的出價
        if self.is_auctioneer:
            self.collected_bids[self.robot_id] = my_utilities

        self.cached_my_bids = my_utilities

        # ★ 改為定時持續廣播 (每 0.5 秒發一次)，防止 Auctioneer 晚啟動而掉包
        self.bid_timer = self.create_timer(0.5, self.broadcast_bid_heartbeat)

    def broadcast_bid_heartbeat(self):
        if self.stage == 'ASSIGNED':
            if hasattr(self, 'bid_timer') and not self.bid_timer.is_canceled():
                self.bid_timer.cancel()
            return

        msg = String()
        msg.data = json.dumps({'id': self.robot_id, 'utilities': self.cached_my_bids})
        self.bid_pub.publish(msg)

    def bid_callback(self, msg):
        if not self.is_auctioneer or self.stage != 'BIDDING':
            return

        data = json.loads(msg.data)
        self.collected_bids[data['id']] = data['utilities']

        # 只要尚未執行過最佳化且已集滿 26 台小車
        if len(self.collected_bids) == self.num_robots and self.stage == 'BIDDING':
            self.stage = 'OPTIMIZING' # 立即鎖定狀態，防止重複計算
            self.get_logger().info("📥 已集齊 26 台小車出價向量，Auctioneer 開始執行匈牙利演算法...")
            
            robot_ids = sorted(list(self.collected_bids.keys()))
            utility_matrix = np.zeros((self.num_robots, self.num_slots))
            for i, r_id in enumerate(robot_ids):
                utility_matrix[i, :] = self.collected_bids[r_id]

            cost_matrix = 2.0 - utility_matrix
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            winners = [-1] * self.num_slots
            for r_idx, s_idx in zip(row_ind, col_ind):
                winners[int(s_idx)] = robot_ids[r_idx]

            # 廣播指派結果
            assign_msg = String()
            assign_msg.data = json.dumps({
                'auctioneer_id': self.robot_id,
                'winners': winners
            })
            self.assignment_pub.publish(assign_msg)
            self.get_logger().info(f"🏆 全域指派完成，已發布名單: {winners}")

def assignment_callback(self, msg):
        if self.stage == 'ASSIGNED':
            return

        data = json.loads(msg.data)
        winners = data['winners']

        if self.robot_id in winners:
            self.assigned_slot = winners.index(self.robot_id)
            
            # 取出 Slot 的相對 offset 與絕對目標座標
            offset_x = self.slot_offsets_x[self.assigned_slot]
            offset_y = self.slot_offsets_y[self.assigned_slot]
            tx = self.target_center_x + offset_x
            ty = self.target_center_y + offset_y

            self.stage = 'ASSIGNED'

            # 停止出價心跳廣播
            if hasattr(self, 'bid_timer') and not self.bid_timer.is_canceled():
                self.bid_timer.cancel()

            self.get_logger().info(
                f"🎯 Robot {self.robot_id} 獲得 Slot {self.assigned_slot} "
                f"-> 目標: ({tx:.2f}, {ty:.2f})，通知運動控制節點出發！"
            )

            # ★ 發布給你的運動控制 Node：[tx, ty, offset_x, offset_y]
            slot_msg = Float32MultiArray()
            slot_msg.data = [float(tx), float(ty), float(offset_x), float(offset_y)]
            self.target_slot_pub.publish(slot_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()