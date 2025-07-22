# Paras.py

# ——— 连杆长度 (单位同 Webots 世界尺度) ———
L1 = 30.1    # Coxa
L2 = 60.91   # Femur
L3 = 152.32  # Tibia

# ——— 电机名称列表（请与 .wbt 中的 DEF 保持一致） ———
motor_names = [
    'leg1_left_coxa',   'leg1_left_femur',   'leg1_left_tibia',
    'leg1_right_coxa',  'leg1_right_femur',  'leg1_right_tibia',
    'leg2_left_coxa',   'leg2_left_femur',   'leg2_left_tibia',
    'leg2_right_coxa',  'leg2_right_femur',  'leg2_right_tibia',
    'leg3_left_coxa',   'leg3_left_femur',   'leg3_left_tibia',
    'leg3_right_coxa',  'leg3_right_femur',  'leg3_right_tibia'
]

# ——— 对应每个电机的初始偏移角（rad）——上面读取的数据———
initial_offsets = {
    'leg1_left_coxa':    0.0000,  'leg1_left_femur':   0.3200,  'leg1_left_tibia':  0.0300,
    'leg1_right_coxa':   0.0000,  'leg1_right_femur': -0.5200,  'leg1_right_tibia': 0.2200,
    'leg2_left_coxa':    0.0000,  'leg2_left_femur':   0.3200,  'leg2_left_tibia':  0.0300,
    'leg2_right_coxa':   0.0000,  'leg2_right_femur': -0.5200,  'leg2_right_tibia': 0.2200,
    'leg3_left_coxa':   -0.0000,  'leg3_left_femur':   0.3200,  'leg3_left_tibia':  0.0300,
    'leg3_right_coxa':   0.0000,  'leg3_right_femur': -0.5200,  'leg3_right_tibia': 0.2200
}

# 新增：每类关节的增量
delta_changes = {
    'femur': 0.6,  # femur 关节的增量
    'tibia': 0.3   # tibia 关节的增量
}

# Paras.py

# …（已有的 L1,L2,L3、motor_names、initial_offsets、delta_changes）…

# 每类关节的增量符号：left 用 -1，right 用 +1
delta_sign = {
    'femur': {'left': -1, 'right': 1},
    'tibia': {'left': -1, 'right': 1}
}
