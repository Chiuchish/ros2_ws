from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 定義系統總共的 Slot 數量（例如 4 個位置）
    num_slots = 4

    # 隊形幾何偏移量字典 (相對編隊中心的 X, Y 座標)
    slot_offsets = [
        [ 0.0,  0.0],  # Slot 0
        [-2.0,  2.0],  # Slot 1
        [-2.0, -2.0],  # Slot 2
        [-2.0,  0.0]   # Slot 3
    ]

    # 所有小車的通訊拓撲與貝氏網路初始出價
    robot_configs = [
        {
            'robot_id': 0,
            'neighbors': [1],
            'utilities': [0.30, 0.40, 0.10, 0.95],
            'init_pos': [1.0, 0.0]  # 初始 X, Y 座標
        },
        {
            'robot_id': 1,
            'neighbors': [0, 2],
            'utilities': [0.20, 0.85, 0.40, 0.15],
            'init_pos': [0.0, 0.5]
        },
        {
            'robot_id': 2,
            'neighbors': [1, 3],
            'utilities': [0.90, 0.50, 0.10, 0.05],
            'init_pos': [0.0, 5.0]
        },
        {
            'robot_id': 3,
            'neighbors': [2],
            'utilities': [0.15, 0.20, 0.90, 0.35],
            'init_pos': [-7.0, -1.0]
        }
    ]

    # 利用 for 迴圈，在單一電腦上動態生成所有小車的「大腦」與「小腦」
    for config in robot_configs:
        r_id = config['robot_id']
        
        # 1. 啟動該小車的【拍賣決策節點（大腦）】
        auction_node = Node(
            package='consensus_test',
            executable='bayesian_auction_node',
            name=f"auction_agent_{r_id}", # 確保單機運行時名稱唯一
            parameters=[{
                'robot_id': r_id,
                'num_slots': num_slots,
                'neighbors': config['neighbors'],
                'utilities': config['utilities']
            }],
            output='screen'
        )
        
        # 2. 啟動該小車的【編隊控制節點（小腦）】
        control_node = Node(
            package='consensus_test',
            executable='control_node', # 指向你剛修改完的控制節點
            name=f"control_agent_{r_id}", # 確保單機運行時名稱唯一
            parameters=[{
                'robot_id': r_id,
                'neighbors': config['neighbors'],
                'initial_value': config['init_pos'], # 傳入二維初始座標 [X, Y]
                # 傳入整套陣型的相對偏移量，供控制節點動態查表
                'slot_offsets_x': [pos[0] for pos in slot_offsets],
                'slot_offsets_y': [pos[1] for pos in slot_offsets]
            }],
            output='screen'
        )
        
        # 將這台小車的兩個核心節點加入啟動清單
        ld.add_action(auction_node)
        ld.add_action(control_node)

    visualizer_node = Node(
        package='consensus_test',
        executable='visualizer_node',
        name='global_visualizer',
        parameters=[{'num_robots': 4}]
    )
    ld.add_action(visualizer_node)        

    return ld
