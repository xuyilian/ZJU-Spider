# multi_leg_phase_both_with_real_offset_plot.py

from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import bezier
import cpg
import Paras
from ik import inverse_kinematics

# ——— 参数 ———
TIMESTEP = 8               # ms
DURATION = 60.0            # s
dt       = TIMESTEP / 1000.0

# 从 Paras 读取
LEG_CONFIG = Paras.LEG_CONFIG_Sidle
Z_LIFT     = Paras.Z_LIFT
Z_DOWN     = Paras.Z_DOWN
leg_sign   = Paras.leg_sign

# ——— CPG 初始化 ———
phases = cpg.initial_phases.copy()  # [φ0, φ1]

# ——— Webots 设备获取 & 初始 offset ———
robot   = Robot()
motors  = {}
sensors = {}
offsets = {}

for leg, cfg in LEG_CONFIG.items():
    motors[leg] = {
        jt: robot.getDevice(f"{leg}_{jt}") for jt in ('coxa','femur','tibia')
    }
    sensors[leg] = {
        jt: robot.getDevice(f"{leg}_{jt}_sensor") for jt in ('coxa','femur','tibia')
    }
    for jt in ('coxa','femur','tibia'):
        sensors[leg][jt].enable(TIMESTEP)
        motors[leg][jt].setVelocity(30.0)

# 读取一次偏移
robot.step(TIMESTEP)
for leg in LEG_CONFIG:
    offsets[leg] = {}
    for jt in ('coxa','femur','tibia'):
        offsets[leg][jt] = sensors[leg][jt].getValue()

# ——— 存储历史数据 ———
times   = []
history = {
    leg: { jt: [] for jt in ('coxa','femur','tibia') }
    for leg in LEG_CONFIG
}

# ——— 主循环 ———
max_steps = int(DURATION * 1000 / TIMESTEP)
t = 0.0
for _ in range(max_steps):
    if robot.step(TIMESTEP) == -1:
        break

    # 1) 更新 CPG 相位
    dy     = cpg.coupled_oscillators2(t, phases)
    phases = [phases[i] + dy[i]*dt for i in (0,1)]
    t += dt
    times.append(t)

    # 2) 对每条腿下发命令并记录
    for leg, cfg in LEG_CONFIG.items():
        osc   = cfg['osc']
        theta = phases[osc] % (2*np.pi)

        # 目标足端
        target = bezier.Bezier(cfg['P1'], cfg['P3'], theta, Z_LIFT, Z_DOWN)
        θ1, θ2, θ3 = inverse_kinematics(target)

        # 加上左右符号 & 偏移
        side = leg.split('_')[-1]  # 'left' or 'right'
        sign = leg_sign[side]
        off  = offsets[leg]
        m    = motors[leg]

        m['coxa'] .setPosition(off['coxa']  + sign * θ1)
        m['femur'].setPosition(off['femur'] + sign * θ2)
        m['tibia'].setPosition(off['tibia'] + sign * θ3)

        # 记录实际读数
        for jt in ('coxa','femur','tibia'):
            history[leg][jt].append(sensors[leg][jt].getValue())

# ——— 仿真结束后，画图 ———
legs = list(LEG_CONFIG.keys())
fig, axes = plt.subplots(len(legs), 3, figsize=(12, 2.5*len(legs)), sharex=True)
for i, leg in enumerate(legs):
    for j, jt in enumerate(('coxa','femur','tibia')):
        ax = axes[i][j]
        ax.plot(times, history[leg][jt], label=f"{leg}_{jt}")
        ax.set_ylabel(f"{jt} (rad)")
        ax.grid(True)
        if i == 0:
            ax.set_title(jt)
    axes[i][0].legend(loc='upper right', fontsize='small')
    axes[i][0].set_ylabel(f"{leg}\nangle")
axes[-1][0].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
