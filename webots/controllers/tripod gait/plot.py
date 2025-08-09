import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import bezier   # 你的 bezier.py
import cpg      # 你的 cpg.py
import Paras
from ik import inverse_kinematics
import Paras

# ——— 仿真参数 ———
TIMESTEP = 0.02   # 秒
DURATION = 10.0   # 总时长，秒
Z_LIFT = 10       # 抬腿高度
Z_DOWN = -5       # 支撑下压


GROUP_A = ['leg2_left', 'leg3_right', 'leg1_right']
GROUP_B = ['leg1_left',  'leg3_left',  'leg2_right']

# CPG 初始化
phases = Paras.initial_phases.copy()  # [φ0, φ1]
dt     = TIMESTEP

# 时间和存储结构
times = np.arange(0, DURATION, dt)
positions   = {leg: {'x': [], 'y': [], 'z': []} for leg in Paras.LEG_CONFIG_Sidle}
joint_angles = {leg: {'coxa': [], 'femur': [], 'tibia': []} for leg in Paras.LEG_CONFIG_Sidle}

# 简单仿真循环（不依赖 Webots）
t = 0.0
for _ in times:
    dy = cpg.coupled_oscillators2(t, phases)
    phases = [phases[i] + dy[i]*dt for i in (0,1)]
    t += dt
    for leg, cfg in Paras.LEG_CONFIG_Sidle.items():
        theta = phases[cfg['osc']] % (2*np.pi)
        pt    = bezier.Bezier(cfg['P1'], cfg['P3'], theta, Z_LIFT, Z_DOWN)
        positions[leg]['x'].append(pt[0])
        positions[leg]['y'].append(pt[1])
        positions[leg]['z'].append(pt[2])
        θ1, θ2, θ3 = inverse_kinematics(pt)
        sign = +1 if leg.endswith('_right') else -1
        # 记录 signed 关节角
        joint_angles[leg]['coxa'].append (sign * θ1)
        joint_angles[leg]['femur'].append(sign * θ2)
        joint_angles[leg]['tibia'].append(sign * θ3)

def plot_group(group, title):
    # —— 1) XYZ vs 时间
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, axis in zip(axes, ('x','y','z')):
        for leg in group:
            ax.plot(times, positions[leg][axis], label=leg)
        ax.set_ylabel(f'{axis}')
        ax.grid(True)
        if axis == 'z':
            ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', fontsize='small')
    fig.suptitle(f'{title} — XYZ vs Time')
    fig.tight_layout(rect=[0,0,1,0.95])

    # —— 2) 3D 轨迹 with colorbar
    fig3d = plt.figure(figsize=(8,6))
    ax3 = fig3d.add_subplot(111, projection='3d')
    norm = plt.Normalize(times.min(), times.max())
    cmap = plt.get_cmap('viridis')
    for leg in group:
        xs = np.array(positions[leg]['x'])
        ys = np.array(positions[leg]['y'])
        zs = np.array(positions[leg]['z'])
        pts = np.stack([xs, ys, zs], axis=1)
        segs = np.concatenate([pts[:-1,None], pts[1:,None]], axis=1)
        lc = Line3DCollection(segs, cmap=cmap, norm=norm)
        lc.set_array(times)
        lc.set_linewidth(2)
        ax3.add_collection(lc)
    all_x = np.hstack([positions[leg]['x'] for leg in group])
    all_y = np.hstack([positions[leg]['y'] for leg in group])
    all_z = np.hstack([positions[leg]['z'] for leg in group])
    ax3.set_xlim(all_x.min(), all_x.max())
    ax3.set_ylim(all_y.min(), all_y.max())
    ax3.set_zlim(all_z.min(), all_z.max())
    ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')
    ax3.set_title(f'{title} — 3D Trajectories w/ Time Color')
    fig3d.colorbar(lc, ax=ax3, pad=0.1, label='Time (s)')

    # —— 3) 关节角 vs 时间
    figj, axj = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joints = ('coxa','femur','tibia')
    for joint, ax in zip(joints, axj):
        for leg in group:
            ax.plot(times, joint_angles[leg][joint], label=leg)
        ax.set_ylabel(f'{joint} angle (rad)')
        ax.grid(True)
        if joint == 'tibia':
            ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', fontsize='small')
    figj.suptitle(f'{title} — Joint Angles vs Time')
    figj.tight_layout(rect=[0,0,1,0.95])

# 分别画两组
plot_group(GROUP_A, 'Group A (osc=0)')
plot_group(GROUP_B, 'Group B (osc=1)')

plt.show()
