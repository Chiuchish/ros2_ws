import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String, Bool
import json
import random
import math
import time

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
        self.declare_parameter('slot_offsets_x', value=[0.0, -1.0, -1.0, -2.0], descriptor=array_desc)
        self.declare_parameter('slot_offsets_y', value=[0.0, 1.0, -1.0, 0.0], descriptor=array_desc)
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


        self.P_K_yes = 0.985
        self.P_K_no = 1.0 - self.P_K_yes

        # 2. 最終勝率 (節點 M) 的條件機率表 (K, L, P)
        self.M_table = {
            ('Yes', 'Yes', 'Low'): 0.999,
            ('Yes', 'Yes', 'High'): 0.600,
            ('Yes', 'No', 'Low'): 0.010,
            ('Yes', 'No', 'High'): 0.001,
            ('No', 'Yes', 'Low'): 0.010,
            ('No', 'Yes', 'High'): 0.001,
            ('No', 'No', 'Low'): 0.001,
            ('No', 'No', 'High'): 0.001,
        }

        self.get_logger().info("正在建立CBN貝氏網路模型...")

        self.my_utilities = self.calculate_bayesian_utilities()

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

    def calculate_bayesian_utilities(self):
        utilities = []
        calc_start_time = time.time()
        
        for slot_idx in range(self.num_slots):
            offset_x = self.slot_offsets_x[slot_idx]
            offset_y = self.slot_offsets_y[slot_idx]
            abs_slot_x = self.target_center_x + offset_x
            abs_slot_y = self.target_center_y + offset_y
            
            distance = math.hypot(abs(self.pos_x - abs_slot_x), abs(self.pos_y - abs_slot_y))
            time_required = (distance / self.max_speed) if self.max_speed > 0 else 99.0
            
            expected_battery = self.current_voltage - (self.voltage_rate * time_required)
            
            # ==========================================
            # ★ 修正重點：確保運算基底大於等於 0
            # ==========================================
            # 1. 時間分數：如果耗時超過 20 秒，分數直接觸底為 0.0
            clamped_time_ratio = max(0.0, 1.0 - (time_required / 25.0))
            time_score = clamped_time_ratio ** 1.5
            
            # 2. 電量分數：同樣加入箝位，確保電量爆表或耗盡時，分數維持在 0.0 ~ 1.0 之間
            clamped_battery = max(10.9, min(12.4, expected_battery))
            battery_score = (clamped_battery - 10.9) / (12.4 - 10.9)

            # 後續機率計算保持不變
            P_L_yes = 0.001 + 0.998 * (0.7 * time_score + 0.3 * battery_score)
            P_L_no = 1.0 - P_L_yes

            if self.sensor_quality == 'Good':
                p_high = 0.01 + 0.02 * time_required
            else:
                p_high = 0.10 + 0.10 * time_required
            
            P_P_high = max(0.0, min(0.999, p_high))
            P_P_low = 1.0 - P_P_high

            P_M_yes = 0.0
        
            K_probs = {'Yes': self.P_K_yes, 'No': self.P_K_no}
            L_probs = {'Yes': P_L_yes, 'No': P_L_no}
            P_probs = {'Low': P_P_low, 'High': P_P_high}

            for k_state, p_k in K_probs.items():
                for l_state, p_l in L_probs.items():
                    for p_state, p_p in P_probs.items():
                        P_M_yes += self.M_table[(k_state, l_state, p_state)] * p_k * p_l * p_p
                        
            try:
                success_prob = P_M_yes
            except Exception as e:
                self.get_logger().error(f"BN 推論失敗: {e}")
                success_prob = 0.0

            tie_breaker = max(0.0, (20.0 - time_required)) * 0.0001
            final_prob = success_prob + tie_breaker

            utilities.append(round(final_prob, 5)) 

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