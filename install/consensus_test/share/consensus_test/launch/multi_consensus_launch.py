from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 定義總位置數量 (如果有 4 台小車，通常會有 4 個陣型 Slot)
    num_slots = 9

    # 使用清單 (List) 集中管理所有機器人的通訊拓撲與貝氏網路機率
    robot_configs = [
        {
            'robot_id': 0,
            'neighbors': [1, 3, 4],
            'utilities': [0.30, 0.40, 0.10, 0.85, 0.50, 0.35, 0.94, 0.14, 0.9]  
        },
        {
            'robot_id': 1,
            'neighbors': [0, 2, 4],
            'utilities': [0.20, 0.85, 0.40, 0.15, 0.80, 0.45, 0.60, 0.55, 0.3]  
        },
        {
            'robot_id': 2,
            'neighbors': [0, 4, 6],
            'utilities': [0.80, 0.85, 0.10, 0.05, 0.90, 0.60, 0.40, 0.65, 0.35]
        },
        {
            'robot_id': 3,
            'neighbors': [0, 2, 4],
            'utilities': [0.15, 0.20, 0.90, 0.95, 0.30, 0.60, 0.20, 0.89, 0.45]
        },
        {
            'robot_id': 4,
            'neighbors': [1, 3, 5, 7],
            'utilities': [0.15, 0.20, 0.90, 0.80, 0.95, 0.55, 0.85, 0.95, 0.45]
        },
        {
            'robot_id': 5,
            'neighbors': [2, 4, 8],
            'utilities': [0.15, 0.20, 0.90, 0.80, 0.80, 0.70, 0.55, 0.90, 0.25]
        },
        {
            'robot_id': 6,
            'neighbors': [3, 4, 7],
            'utilities': [0.01, 0.20, 0.90, 0.80, 0.25, 0.70, 0.95, 0.30, 0.65]
        },
        {
            'robot_id': 7,
            'neighbors': [4, 6, 8],
            'utilities': [0.95, 0.20, 0.60, 0.80, 0.25, 0.70, 0.05, 0.30, 0.45]
        },
        {
            'robot_id': 8,
            'neighbors': [5, 7],
            'utilities': [0.90, 0.20, 0.90, 0.80, 0.25, 0.70, 0.05, 0.30, 0.95]
        }
    ]

    # 利用 for 迴圈動態建立所有節點 (Agents)
    for config in robot_configs:
        node = Node(
            package='consensus_test',
            executable='bayesian_auction_node',
            name=f"auction_agent_{config['robot_id']}",
            parameters=[{
                'robot_id': config['robot_id'],
                'num_slots': num_slots,
                'neighbors': config['neighbors'],
                'utilities': config['utilities']
            }]
        )
        ld.add_action(node)

    return ld
