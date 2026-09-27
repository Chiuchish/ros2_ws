import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import random

class CentralizedAgentNode(Node):
    def __init__(self):
        super().__init__('centralized_agent')
        
        # 1. 讀取與原本相同的物理參數
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_quality', 'Good')

        self.robot_id = self.get_parameter('robot_id').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensor_quality = self.get_parameter('sensor_quality').value
        self.current_voltage = 10.7
        
        # 儲存自身最新座標
        self.current_x = 0.0
        self.current_y = 0.0

        # 2. 訂閱自己的小腦，獲取真實物理座標
        self.create_subscription(
            Float32MultiArray, f'/robot_{self.robot_id}/kinematic_state', self.kinematic_callback, 10
        )

        # 3. 向上呈報通道 (給指揮官)
        self.telemetry_pub = self.create_publisher(String, '/central_telemetry', 10)
        self.create_timer(1.0, self.report_telemetry)

        # 4. 接收聖旨通道 (來自指揮官)
        self.create_subscription(String, '/central_assignment', self.assignment_callback, 10)

        # 5. 向下發布通道 (給自己的小腦)
        self.local_brain_pub = self.create_publisher(String, f'/robot_{self.robot_id}/auction_state', 10)
        self.get_logger().info(f"Robot {self.robot_id} (中心模式) 啟動，等待指揮官指示...")

    def kinematic_callback(self, msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]

    def report_telemetry(self):
        info = {
            'id': self.robot_id,
            'pos_x': self.current_x,
            'pos_y': self.current_y,
            'max_speed': self.max_speed,
            'voltage': self.current_voltage,
            'sensor_quality': self.sensor_quality
        }
        msg = String()
        msg.data = json.dumps(info)
        self.telemetry_pub.publish(msg)

    def assignment_callback(self, msg):
        data = json.loads(msg.data)
        winners = data.get('winners', [])
        
        # 偽裝成原本分散式大腦的輸出格式，欺騙小腦
        local_msg = String()
        local_msg.data = json.dumps({
            'bids': [1.0] * len(winners), # 中心式不在乎出價
            'winners': winners,
            'is_consensus': True          # 中心式永遠是強制完美共識
        })
        self.local_brain_pub.publish(local_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CentralizedAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()