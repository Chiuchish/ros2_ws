import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class FleetVisualizerNode(Node):
    def __init__(self):
        super().__init__('fleet_visualizer')
        
        # 宣告要監控的機器人總數
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # 建立一個字典來儲存所有機器人的最新座標，預設在原點附近
        self.robot_positions = {i: [0.0, 0.0] for i in range(self.num_robots)}
        
        # 動態為每台機器人建立訂閱者，接收小腦發出的 kinematic_state
        for i in range(self.num_robots):
            topic = f'/robot_{i}/kinematic_state'
            self.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg, r_id=i: self.state_callback(msg, r_id),
                10
            )
            self.get_logger().info(f'視覺化監控已訂閱: {topic}')

    def state_callback(self, msg, robot_id):
        """當收到機器人的座標更新時，寫入記憶體"""
        # msg.data 格式為 [本體X, 本體Y, 目標偏移X, 目標偏移Y]
        x = msg.data[0]
        y = msg.data[1]
        self.robot_positions[robot_id] = [x, y]

def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualizerNode()
    
    # ---------------------------------------------------------
    # 技巧：將 ROS 2 的 spin() 丟到背景執行緒，以免卡死畫圖視窗
    # ---------------------------------------------------------
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ---------------------------------------------------------
    # Matplotlib 畫圖設定 (在主執行緒執行)
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 8)) # 設定視窗大小
    
    # 為每台機器人設定不同的顏色，方便辨識
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    
    def update_plot(frame):
        ax.clear()
        # 設定畫布範圍，視你的初始座標與隊形大小而定
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect('equal') # 確保 X 軸和 Y 軸比例 1:1，隊形才不會變形
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_title(f'(Agents: {node.num_robots})', fontsize=14)
        ax.set_xlabel('X  (meter)')
        ax.set_ylabel('Y  (meter)')
        
        # 畫出每台機器人的位置
        for r_id, pos in node.robot_positions.items():
            x, y = pos[0], pos[1]
            color = colors[r_id % len(colors)] # 循環取色
            
            # 畫圓點代表小車
            ax.scatter(x, y, c=color, s=100, edgecolors='black', zorder=5)
            # 在圓點旁邊標示 Robot ID
            ax.text(x + 0.15, y + 0.15, f'R{r_id}', fontsize=12, fontweight='bold', color='black')
            
    # 使用 FuncAnimation 建立動畫，每 100 毫秒 (10Hz) 更新一次畫面
    ani = animation.FuncAnimation(fig, update_plot, interval=10)
    
    # 顯示視窗 (這行會卡住主程式，直到使用者手動關閉視窗)
    plt.show()

    # 當視窗被關閉後，優雅地關閉 ROS 2 節點
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
