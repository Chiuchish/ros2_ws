from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. 系統總共的 Slot 數量
    num_slots = 7

    # 2. 全域目標中心點 (Global Target Center)
    # 這代表整個隊形要以這個點為基準展開
    target_center_x = 0.0
    target_center_y = 0.0

    # 3. 隊形幾何偏移量 (相對編隊中心的 X, Y 座標)
    slot_offsets = [
        [ 0.0,  0.0],  # Slot 0 
        [-1.0,  1.0],  # Slot 1 
        [-1.0, -1.0],  # Slot 2 
        [-2.0,  0.0],  # Slot 3
        [-1.0,  0.0],  # Slot 4
        [-2.0,  1.0],  # Slot 5
        [-2.0, -1.0],  # Slot 6
    ]
    
    slot_offsets_x = [pos[0] for pos in slot_offsets]
    slot_offsets_y = [pos[1] for pos in slot_offsets]

    robot_configs = [
        {
            'robot_id': 0,
            'neighbors': [1, 4],
            'init_pos': [0.0, 3.0],   
            'max_speed': 1.5          
        },
        {
            'robot_id': 1,
            'neighbors': [0, 2, 4],
            'init_pos': [-3.0, 3.0],
            'max_speed': 0.8          
        },
        {
            'robot_id': 2,
            'neighbors': [1, 3, 4],
            'init_pos': [3.0, -3.0],
            'max_speed': 1.2          
        },
        {
            'robot_id': 3,
            'neighbors': [2, 4],
            'init_pos': [-4.0, -4.0],
            'max_speed': 2.0          
        },
        {
            'robot_id': 4,
            'neighbors': [0, 1, 2, 3, 5, 6],
            'init_pos': [-4.0, 8.0],
            'max_speed': 0.7          
        },
        {
            'robot_id': 5,
            'neighbors': [4, 6],
            'init_pos': [0.0, 1.0],
            'max_speed': 0.2          
        },
        {
            'robot_id': 6,
            'neighbors': [4, 5],
            'init_pos': [-4.0, 0.0],
            'max_speed': 1.3          
        }                
    ]

    for config in robot_configs:
        r_id = config['robot_id']
        
        # ==========================================
        # 啟動【大腦】：貝氏網路拍賣節點
        # ==========================================
        auction_node = Node(
            package='consensus_test',
            executable='Parameterized_bayesian_node', # 請確認是否對應您 setup.py 裡的名稱
            name=f"auction_agent_{r_id}",
            parameters=[{
                'robot_id': r_id,
                'num_slots': num_slots,
                'neighbors': config['neighbors'],
                'max_speed': config['max_speed'],        
                'init_pos_x': config['init_pos'][0],     
                'init_pos_y': config['init_pos'][1],
                # 【新增】將全域目標傳給大腦
                'target_center_x': target_center_x,
                'target_center_y': target_center_y,
                'slot_offsets_x': slot_offsets_x,        
                'slot_offsets_y': slot_offsets_y
            }],
            output='screen'
        )
        
        # ==========================================
        # 啟動【小腦】：編隊物理控制節點
        # ==========================================
        control_node = Node(
            package='consensus_test',
            executable='control_limit_node', # 請確認是否對應您 setup.py 裡的名稱
            name=f"control_agent_{r_id}",
            parameters=[{
                'robot_id': r_id,
                'neighbors': config['neighbors'],
                'initial_value': config['init_pos'],     
                'max_speed': config['max_speed'],
                # 注意：小腦的 offset 也需要跟著改變，
                # 但因為我們在 control_node 中沒有去修改絕對座標基準點，
                # 我們可以直接把 offset 加上 target_center，讓小腦認為「虛擬彈簧的錨點」就在絕對位置上。
                'slot_offsets_x': [x + target_center_x for x in slot_offsets_x],        
                'slot_offsets_y': [y + target_center_y for y in slot_offsets_y]
            }],
            output='screen'
        )
        
        ld.add_action(auction_node)
        ld.add_action(control_node)

    # 監控面板 (請記得需要獨立終端機啟動時可先註解這段)
    visualizer_node = Node(
        package='consensus_test',
        executable='visualizer_node',
        name='global_visualizer',
        parameters=[{'num_robots': 7}],
        output='screen'
    )
    ld.add_action(visualizer_node)    

    return ld