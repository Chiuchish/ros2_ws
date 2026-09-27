import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String, Bool
import json

class BayesianAuctionNode(Node):
    def __init__(self):
        super().__init__('bayesian_auction_agent')

        # ==========================================
        # 1. 參數宣告與讀取
        # ==========================================
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_slots', 4)
        
        neighbor_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        self.declare_parameter('neighbors', descriptor=neighbor_desc)
        
        utility_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('utilities', descriptor=utility_desc)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_slots = self.get_parameter('num_slots').value
        self.neighbors = self.get_parameter('neighbors').value or []
        self.my_utilities = self.get_parameter('utilities').value or []

        # ==========================================
        # 2. 內部記憶體與狀態控制
        # ==========================================
        self.highest_bids = [0.0] * self.num_slots
        self.winners = [-1] * self.num_slots
        
        self.cycle_count = 0
    
        # 【新增】靜默與共識控制變數
        self.idle_cycles = 0           # 連續沒有發生改變的週期數
        self.consensus_reached = False # 是否已經達成共識
        self.changed_by_neighbor = False # 這個週期是否有收到鄰居的新資訊

        # ==========================================
        # 3. P2P 通訊設定
        # ==========================================
        pub_topic = f'/robot_{self.robot_id}/auction_state'
        self.publisher_ = self.create_publisher(String, pub_topic, 10)

        for j in self.neighbors:
            sub_topic = f'/robot_{j}/auction_state'
            self.create_subscription(
                String, sub_topic, 
                lambda msg, n_id=j: self.neighbor_callback(msg, n_id), 10
            )
            
        # 【新增】外部喚醒訂閱者 (模擬貝氏網路狀態改變，需要重新分配)
        self.create_subscription(Bool, f'/robot_{self.robot_id}/trigger_reallocation', self.trigger_callback, 10)

        # 4. 決策心跳 (1Hz)
        self.timer = self.create_timer(0.1, self.auction_loop)
        self.get_logger().info(f"Robot {self.robot_id} 啟動！等待達成共識...")

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
                    
            # 【關鍵修正】只有當收到的訊息「真的改變了本地認知」時，才標記改變並解除靜默
            if changed:
                self.changed_by_neighbor = True
                # 如果原本處於靜默狀態，被有效的改變打斷，才重新啟動協商
                if self.consensus_reached:
                    self.consensus_reached = False
                    self.idle_cycles = 0
                    self.get_logger().info(f"⚠️ 收到 Robot {neighbor_id} 的【有效】新狀態，解除靜默，重啟協商！")
        except Exception as e:
            pass

    def trigger_callback(self, msg):
        """【喚醒機制】模擬內部貝氏網路狀態劇烈變化"""
        if msg.data:
            self.get_logger().warn(f"🚨 內部狀態改變 (例如電量過低)！強制打破共識，請求重新分配！")
            
            # 模擬: 強制降低自己目前的出價，或者修改 utilities (這裡簡化為修改自身出價並喚醒)
            # 在實際研究中，這裡是貝氏網路重算 my_utilities 的地方
            
            self.consensus_reached = False
            self.idle_cycles = 0
            self.changed_by_neighbor = True # 欺騙系統發生改變，強制重啟廣播

    def auction_loop(self):
        """主決策迴圈"""
        # 1. 如果已經達成共識，維持靜默
        if self.consensus_reached:
            return

        self.cycle_count += 1
        local_changed = self.run_local_auction()
        
        if not local_changed and not self.changed_by_neighbor:
            self.idle_cycles += 1
        else:
            self.idle_cycles = 0 
            
        self.changed_by_neighbor = False 
        
        # 2. 共識判定邏輯
        is_consensus = False
        if self.idle_cycles >= 3 and -1 not in self.winners:
            self.consensus_reached = True
            is_consensus = True
            self.get_logger().info(f"✅ 達成完美共識！發送最終分配結果並進入靜默模式。")

        # 3. 將 is_consensus 訊號一起打包廣播
        state_dict = {
            'bids': self.highest_bids,
            'winners': self.winners,
            'is_consensus': is_consensus  # <--- 新增的共識訊號
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