import json
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. 系統總共的 Slot 數量
    num_slots = 26

    # 2. 全域目標中心點 (Global Target Center)
    # 這代表整個隊形要以這個點為基準展開
    target_center_x = 0.0
    target_center_y = 0.0

    # 3. 隊形幾何偏移量 (相對編隊中心的 X, Y 座標)
    slot_offsets = [
        [ 0.0,  0.0],  # Slot 0 
        [-2.5,  2.5],  # Slot 1 
        [-2.5,  0.0],  # Slot 2 
        [-2.5, -2.5],  # Slot 3
        [ 0.0, -2.5],  # Slot 4
        [ 2.5,  2.5],  # Slot 5
        [ 5.0,  2.5],  # Slot 6
        [ 5.0,  0.0],  # Slot 7
        [ 5.0, -2.5],  # Slot 8
        [-2.5,  1.25],  # Slot 9
        [-2.5, -1.25],  # Slot 10
        [-1.25, -2.5],  # Slot 11
        [-1.25,  0.0],  # Slot 12
        [ 0.0, -1.25],  # Slot 13
        [ 3.75,  2.5],  # Slot 14
        [ 5.0,  1.25],  # Slot 15
        [ 5.0, -1.25],  # Slot 16
        [-1.25,  2.5],  # Slot 17
        [ 0.0,  2.5],  # Slot 18
        [ 0.0,  1.25],  # Slot 19
        [ 2.5,  1.25],  # Slot 20
        [ 2.5,  0.0],  # Slot 21
        [ 2.5, -1.25],  # Slot 22
        [ 2.5, -2.5],  # Slot 23
        [ 3.75,  0.0],  # Slot 24
        [ 3.75, -2.5],  # Slot 25
    ]
    
    slot_offsets_x = [pos[0] for pos in slot_offsets]
    slot_offsets_y = [pos[1] for pos in slot_offsets]

    robot_configs = [
        {
            'robot_id': 0,
            'neighbors': [1, 25],
            'init_pos': [0.0, 3.0],   
            'max_speed': 1.5,
            'sensor_quality': 'Poor'          
        },
        {
            'robot_id': 1,
            'neighbors': [0, 2],
            'init_pos': [-3.0, 3.0],
            'max_speed': 0.8,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 2,
            'neighbors': [1, 3],
            'init_pos': [3.0, -3.0],
            'max_speed': 1.2,
            'sensor_quality': 'Poor'          
        },
        {
            'robot_id': 3,
            'neighbors': [2, 4],
            'init_pos': [-4.0, -4.0],
            'max_speed': 2.0,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 4,
            'neighbors': [3, 5],
            'init_pos': [-4.0, 8.0],
            'max_speed': 0.7,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 5,
            'neighbors': [4, 6],
            'init_pos': [0.0, 1.0],
            'max_speed': 0.2,        
            'sensor_quality': 'Good'  
        },
        {
            'robot_id': 6,
            'neighbors': [5, 7],
            'init_pos': [-4.0, 0.0],
            'max_speed': 1.3,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 7,
            'neighbors': [6, 8],
            'init_pos': [-3.0, 0.0],
            'max_speed': 0.4,
            'sensor_quality': 'Poor'          
        },
        {
            'robot_id': 8,
            'neighbors': [7, 9],
            'init_pos': [0.0, 7.0],
            'max_speed': 1.5,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 9,
            'neighbors': [8, 10],
            'init_pos': [0.5, 3.5],
            'max_speed': 1.8,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 10,
            'neighbors': [9, 11],
            'init_pos': [1.25, -7.0],
            'max_speed': 1.0,
            'sensor_quality': 'Poor'          
        },                
        {
            'robot_id': 11,
            'neighbors': [10, 12],
            'init_pos': [3.0, 7.0],
            'max_speed': 3.0,
            'sensor_quality': 'Good'          
        },                
        {
            'robot_id': 12,
            'neighbors': [11, 13],
            'init_pos': [-1.0, 2.0],
            'max_speed': 0.6,
            'sensor_quality': 'Poor'          
        },      
        {
            'robot_id': 13,
            'neighbors': [12, 14],
            'init_pos': [2.0, -4.0],
            'max_speed': 1.2,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 14,
            'neighbors': [13, 15],
            'init_pos': [-5.0, 1.0],
            'max_speed': 1.1,       
            'sensor_quality': 'Good'   
        },
        {
            'robot_id': 15,
            'neighbors': [14, 16],
            'init_pos': [0.0, 0.0],
            'max_speed': 4.0, 
            'sensor_quality': 'Poor'         
        },
        {
            'robot_id': 16,
            'neighbors': [15, 17],
            'init_pos': [3.0, -4.0],
            'max_speed': 0.9,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 17,
            'neighbors': [16, 18],
            'init_pos': [-1.5, -3.0],
            'max_speed': 0.8,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 18,
            'neighbors': [17, 19],
            'init_pos': [-3.5, -2.0],
            'max_speed': 1.0,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 19,
            'neighbors': [18, 20],
            'init_pos': [-2.5, -5.0],
            'max_speed': 1.1,
            'sensor_quality': 'Poor'          
        },
        {
            'robot_id': 20,
            'neighbors': [19, 21],
            'init_pos': [4.0, -1.0],
            'max_speed': 1.2,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 21,
            'neighbors': [20, 22],
            'init_pos': [-5.0, -6.0],
            'max_speed': 1.5,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 22,
            'neighbors': [21, 23],
            'init_pos': [6.0, 1.0],
            'max_speed': 1.4,
            'sensor_quality': 'Good'          
        },        
        {
            'robot_id': 23,
            'neighbors': [22, 24],
            'init_pos': [4.0, -4.0],
            'max_speed': 1.9,
            'sensor_quality': 'Poor'          
        },
        {
            'robot_id': 24,
            'neighbors': [23, 25],
            'init_pos': [7.0, 3.0],
            'max_speed': 3.0,
            'sensor_quality': 'Good'          
        },
        {
            'robot_id': 25,
            'neighbors': [24, 0],
            'init_pos': [4.0, 1.5],
            'max_speed': 2.0,
            'sensor_quality': 'Good'          
        },
    ]
    robot_configs_str = json.dumps({str(cfg['robot_id']): cfg for cfg in robot_configs})

    for config in robot_configs:
        r_id = config['robot_id']
        
        # ==========================================
        # 啟動【大腦】：貝氏網路拍賣節點
        # ==========================================
        agent_node = Node(
            package='centralized_op',
            executable='agent_node', # 請確認是否對應您 setup.py 裡的名稱
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
                'slot_offsets_y': slot_offsets_y,
                'sensor_quality': config['sensor_quality']
            }],
            output='screen'
        )
        
        # ==========================================
        # 啟動【小腦】：編隊物理控制節點
        # ==========================================
        control_node = Node(
            package='centralized_op',
            executable='control_node', # 請確認是否對應您 setup.py 裡的名稱
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
                'slot_offsets_y': [y + target_center_y for y in slot_offsets_y],
                'sensor_quality': config['sensor_quality']
            }],
            output='screen'
        )
        
        ld.add_action(agent_node)
        ld.add_action(control_node)

    # 監控面板 (請記得需要獨立終端機啟動時可先註解這段)
    visualizer_node = Node(
        package='centralized_op',
        executable='visualizer_node',
        name='global_visualizer',
        parameters=[{
            'num_robots': len(robot_configs),     # 自動計算為 26 台
            'speed_threshold': 1.0,                # 區分快車與慢車的門檻
            'k_nearest': 2,
            'robot_configs_json': robot_configs_str,
            'slot_offsets_x': [0.0] * len(robot_configs),  # 依實際 slot 配置替換
            'slot_offsets_y': [0.0] * len(robot_configs),
                     }],
        output='screen'
    )
    ld.add_action(visualizer_node)    

    auction_node = Node(
        package='centralized_op',
        executable='central_node',
        name='auctioneer',
        parameters=[{
            'num_slots': num_slots,
            'num_robots': 26,
            'max_speed': config['max_speed'],        
            'init_pos_x': config['init_pos'][0],     
            'init_pos_y': config['init_pos'][1],
            # 【新增】將全域目標傳給大腦
            'target_center_x': target_center_x,
            'target_center_y': target_center_y,
            'slot_offsets_x': slot_offsets_x,        
            'slot_offsets_y': slot_offsets_y,
            'sensor_quality': config['sensor_quality']
        }],
        output='screen'
    )
    ld.add_action(auction_node)    

    evaluator_node = Node(
        package='centralized_op',
        executable='evaluator_node',
        name='distance_evaluator',
        parameters=[{
            'num_robots': len(robot_configs),
            'robot_configs_json': robot_configs_str,
            'output_path': 'sensor_distance_comparison.png'
        }],
        output='screen'
    )
    ld.add_action(evaluator_node)

    return ld