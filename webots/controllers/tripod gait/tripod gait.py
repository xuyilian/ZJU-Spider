from controller import Robot
import math
import numpy as np
import matplotlib.pyplot as plt

import bezier
import cpg
import Paras
from ik import inverse_kinematics

# ——— 参数 ———
TIMESTEP = 2               # ms
DURATION = 20.0            # s
dt       = TIMESTEP / 1000.0

# 从 Paras 读取
LEG_CONFIG = Paras.LEG_CONFIG_Forward
Z_LIFT     = Paras.Z_LIFT
Z_DOWN     = Paras.Z_DOWN
leg_sign   = Paras.leg_sign

# ——— CPG 初始化 ———
phases = Paras.initial_phases_2.copy()   # 长度6

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
        motors[leg][jt].setVelocity(100.0)

# === 整体速度/角速度传感器 ===
gps  = robot.getDevice('gps')
imu  = robot.getDevice('imu')   # 注意：需与 .wbt 中设备名一致（若默认是 "inertial unit" 请改名）
gyro = robot.getDevice('gyro')
gps.enable(TIMESTEP)
imu.enable(TIMESTEP)
gyro.enable(TIMESTEP)

def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0, 0, 1]])
    Ry = np.array([[cp, 0, sp],[0, 1, 0],[-sp, 0, cp]])
    Rx = np.array([[1, 0, 0],[0, cr, -sr],[0, sr, cr]])
    return Rz @ Ry @ Rx  # world_from_body

# 读取一次偏移
robot.step(TIMESTEP)
for leg in LEG_CONFIG:
    offsets[leg] = {}
    for jt in ('coxa','femur','tibia'):
        offsets[leg][jt] = sensors[leg][jt].getValue()

# === 记录器 ===
times   = []
history = {leg: {jt: [] for jt in ('coxa','femur','tibia')} for leg in LEG_CONFIG}

# 相位历史
phase_hist = []   # 每步记录长度为6的相位数组

# 速度/角速度历史
v_world_hist = []   # [vx, vy, vz] in world
v_body_hist  = []   # [vx, vy, vz] in body
omega_hist   = []   # [wx, wy, wz] from gyro (body)

# 记录 gait ω(t) 与 φ(t)
omega_traj = []
phi_traj   = []

# 初始化速度差分
prev_pos = np.array(gps.getValues(), dtype=float)

# ——— omega 随时间变化（给 CPG/步态用）———
OMEGA_MIN = 0.6
OMEGA_MAX = 1.0
RAMP_TIME = 10.0  # 秒内爬升到上限
def omega_of_t(t):
    if RAMP_TIME <= 0: return OMEGA_MAX
    s = min(max(t / RAMP_TIME, 0.0), 1.0)
    return OMEGA_MIN + s * (OMEGA_MAX - OMEGA_MIN)

# 与 Bezier_2 一致的 φ(ω) 计算（π/3 → π 线性映射，并钳制）
def phi_of_omega(omega, omega_min=0.6, omega_max=1.0,
                 phi_min=np.pi/3.0, phi_max=np.pi):
    if omega_max == omega_min:
        w = 1.0
    else:
        w = (omega - omega_min) / (omega_max - omega_min)
    w = np.clip(w, 0.0, 1.0)
    return phi_min + w * (phi_max - phi_min)

# 方法2：单独速度采样间隔（每 N 个控制周期更新一次速度）
VEL_UPDATE_INTERVAL = 1           # 例：每 10 步（= 20ms）更新一次
counter = 0
last_v_world = np.zeros(3)
last_v_body  = np.zeros(3)
last_omega   = np.zeros(3)

# ——— 主循环 ———
max_steps = int(DURATION * 1000 / TIMESTEP)
t = 0.0
for _ in range(max_steps):
    if robot.step(TIMESTEP) == -1:
        break

    # 1) 更新 CPG 相位（你当前用的可变耦合函数）
    omega_t = omega_of_t(t)
    dy      = cpg.coupled_oscillators_group6_variable(t, phases)  # 若用别的CPG函数可替换
    phases  = [phases[i] + dy[i]*dt for i in range(6)]
    t += dt
    times.append(t)

    # 记录相位与 gait ω、以及 φ(ω)
    phase_hist.append(phases.copy())
    omega_traj.append(omega_t)
    phi_traj.append(phi_of_omega(omega_t, omega_min=OMEGA_MIN, omega_max=OMEGA_MAX))

    # 2) 计算/记录 v_world, v_body, omega_body（按方法2间隔更新）
    counter += 1
    if counter % VEL_UPDATE_INTERVAL == 0:
        pos = np.array(gps.getValues(), dtype=float)
        effective_dt = dt * VEL_UPDATE_INTERVAL
        v_world = (pos - prev_pos) / effective_dt
        prev_pos = pos

        roll, pitch, yaw = imu.getRollPitchYaw()
        R_wb = rpy_to_R(roll, pitch, yaw)
        v_body = R_wb.T @ v_world

        omega_body = np.array(gyro.getValues(), dtype=float)

        last_v_world = v_world
        last_v_body  = v_body
        last_omega   = omega_body
    else:
        v_world     = last_v_world
        v_body      = last_v_body
        omega_body  = last_omega

    v_world_hist.append(np.asarray(v_world).tolist())
    v_body_hist.append(np.asarray(v_body).tolist())
    omega_hist.append(np.asarray(omega_body).tolist())

    # 3) 对每条腿下发命令并记录
    for leg, cfg in LEG_CONFIG.items():
        osc   = cfg['osc']
        theta = phases[osc] % (2*np.pi)

        # 注意：Bezier_2 需实现为带 omega 的版本（你问题里给出的那段）
        target = bezier.Bezier_2(cfg['P1'], cfg['P3'], theta, Z_LIFT, Z_DOWN,
                                 omega=omega_t, omega_min=OMEGA_MIN, omega_max=OMEGA_MAX)
        θ1, θ2, θ3 = inverse_kinematics(target)

        side = leg.split('_')[-1]  # 'left' or 'right'
        sign = leg_sign[side]
        off  = offsets[leg]
        m    = motors[leg]

        m['coxa'] .setPosition(off['coxa']  + sign * θ1)
        m['femur'].setPosition(off['femur'] + sign * θ2)
        m['tibia'].setPosition(off['tibia'] + sign * θ3)

        for jt in ('coxa','femur','tibia'):
            history[leg][jt].append(sensors[leg][jt].getValue())

# === 绘图部分 ===
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

phase_arr = np.array(phase_hist)                  
phase_wrapped = np.mod(phase_arr, 2*np.pi)
v_body_arr = np.array(v_body_hist)
omega_arr = np.array(omega_hist)


fig, axes = plt.subplots(9, 1, figsize=(100, 10), sharex=True)

# 1) omega(t)
axes[0].plot(times, omega_traj, label="omega(t)")
axes[0].set_ylabel("rad/s")
axes[0].set_title("Gait frequency omega vs time")
axes[0].grid(True)
axes[0].legend()

# 2) phi(t)
axes[1].plot(times, phi_traj, label="phi(omega(t))")
axes[1].set_ylabel("phi (rad)")
axes[1].set_title("Bezier phase split φ(t)")
axes[1].grid(True)
axes[1].legend()

# 3) 相位曲线
labels = [f"osc {i}" for i in range(6)]
for i in range(6):
    axes[2].plot(times, phase_wrapped[:, i], label=labels[i])
axes[2].set_ylabel("Phase (rad)")
axes[2].set_title("Oscillator phases vs time (wrapped)")
axes[2].grid(True)
axes[2].legend(ncol=3)

# 4-6) v_body
axes[3].plot(times, v_body_arr[:,0], label="v_body_x", color='red')
axes[3].set_ylabel("m/s")
axes[3].set_title("Body frame velocity X")
axes[3].grid(True)
axes[3].legend()

axes[4].plot(times, v_body_arr[:,1], label="v_body_y", color='blue')
axes[4].set_ylabel("m/s")
axes[4].set_title("Body frame velocity Y")
axes[4].grid(True)
axes[4].legend()

axes[5].plot(times, v_body_arr[:,2], label="v_body_z", color='green')
axes[5].set_ylabel("m/s")
axes[5].set_title("Body frame velocity Z")
axes[5].grid(True)
axes[5].legend()

# 7-9) omega_body
axes[6].plot(times, omega_arr[:,0], label="omega_x", color='orange')
axes[6].set_ylabel("rad/s")
axes[6].set_title("Body angular velocity X")
axes[6].grid(True)
axes[6].legend()

axes[7].plot(times, omega_arr[:,1], label="omega_y", color='purple')
axes[7].set_ylabel("rad/s")
axes[7].set_title("Body angular velocity Y")
axes[7].grid(True)
axes[7].legend()

axes[8].plot(times, omega_arr[:,2], label="omega_z", color='brown')
axes[8].set_ylabel("rad/s")
axes[8].set_title("Body angular velocity Z")
axes[8].set_xlabel("Time (s)")
axes[8].grid(True)
axes[8].legend()

plt.tight_layout()
plt.show()