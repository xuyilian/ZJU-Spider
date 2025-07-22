# multi_leg_two_phases_fixed.py

from controller import Robot
import numpy as np
import bezier
import cpg
import Paras
from ik import inverse_kinematics

# ——— 参数 ———
TIMESTEP = 32  # ms
Z_LIFT = 10    # 抬腿高度
Z_DOWN = -10   # 支撑下压

# 为所有需要驱动的腿，指定它用哪个振荡器 (0 或 1) 以及它的 P1/P3
LEG_CONFIG = {
    # 振荡器 0 驱动的三条腿
    'leg2_left':  {'osc': 0, 'P1': [167.36, -60.0, -89.84], 'P3': [167.36,  60.0, -89.84]},
    'leg3_right': {'osc': 0, 'P1': [167.36+30*np.sqrt(3), -30.0, -89.84],
                               'P3': [167.36-30*np.sqrt(3),  30.0,  -89.84]},
    'leg1_right': {'osc': 0, 'P1': [167.36-30*np.sqrt(3), -30.0, -89.84],
                               'P3': [167.36+30*np.sqrt(3),  30.0,  -89.84]},
    # 振荡器 1 驱动的三条腿
    'leg2_right': {'osc': 1, 'P1': [167.36,  -60.0, -89.84], 'P3': [167.36,  60.0, -89.84]},
    'leg3_left':  {'osc': 1, 'P1': [167.36+30*np.sqrt(3), -30.0, -89.84],
                               'P3': [167.36-30*np.sqrt(3),  30.0, -89.84]},
    'leg1_left':  {'osc': 1, 'P1': [167.36-30*np.sqrt(3), -30.0, -89.84],
                               'P3': [167.36+30*np.sqrt(3),  30.0, -89.84]},
}

# ——— CPG 初始化 ———
y  = cpg.initial_phases.copy()  # [φ0, φ1]
dt = TIMESTEP / 1000.0

# ——— Webots 设备获取 ———
robot = Robot()
motors  = {}
offsets = {}

for leg, cfg in LEG_CONFIG.items():
    # motor、sensor 名字都一样的模式
    motors[leg] = {
        jt: robot.getDevice(f"{leg}_{jt}") for jt in ('coxa','femur','tibia')
    }
    # 只要 PositionSensor 来打印验证，不再用它做偏移
    #offsets 从 Paras 中读
    offsets[leg] = {
        jt: Paras.initial_offsets[f"{leg}_{jt}"] for jt in ('coxa','femur','tibia')
    }
    # 打开速度控制器
    for m in motors[leg].values():
        m.setVelocity(1.0)

# ——— 主循环 ———
t = 0.0

while robot.step(TIMESTEP) != -1:
    # 1) 更新振荡器
    dy = cpg.coupled_oscillators2(t, y)
    y  = [y[i] + dy[i]*dt for i in (0,1)]
    t += dt

    # 2) 对每条腿，根据它配置的 osc 索引选相位，生成目标、IK、下发
    for leg, cfg in LEG_CONFIG.items():
        osc_idx = cfg['osc']
        theta   = y[osc_idx] % (2*np.pi)
        target  = bezier.Bezier(cfg['P1'], cfg['P3'], theta, Z_LIFT, Z_DOWN)
        θ1, θ2, θ3 = inverse_kinematics(target)

        sign = +1 if leg.endswith('_right') else -1
        off  = offsets[leg]
        m    = motors[leg]

        # coxa/femur/tibia 一致处理
        m['coxa'] .setPosition(off['coxa']  + sign*θ1)
        m['femur'].setPosition(off['femur'] + sign*θ2)
        m['tibia'].setPosition(off['tibia'] + sign*θ3)
