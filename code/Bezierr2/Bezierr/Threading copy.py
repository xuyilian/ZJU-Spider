#!/usr/bin/python
# -*- coding: UTF-8 -*-

import threading
import time

from Init import insec, get_angle_from_site, STM32ServoController
from BezierMapping import map_to_joints, calculate_current_site
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 用于3D绘图
from matplotlib.animation import FuncAnimation
from STM32CommunicateInt import send_cmd, get_joint_cmd
from Paras import Paras

# 全局数据缓存
angles = [[[], [], [], []] for _ in range(3)]      # 三关节 × 四腿 历史角度
t = []                                             # 时间戳序列
CPGSignals = [[] for _ in range(4)]                # 振荡器原始相位
sawSignals = [[] for _ in range(4)]                # 振荡器归一化相位
controlSignals = [[] for _ in range(4)]            # 控制相位
control_angles = [[1500]*4 for _ in range(3)]      # 当前 PWM 指令缓存
ppos_all = [[[] for _ in range(4)] for _ in range(3)]  # 足端轨迹: X/Y/Z × 4腿

initial_positions = [
    [0,   0,   Paras.Stand_Site[2]],
    [100,  0,   Paras.Stand_Site[2]],
    [100, -100,  Paras.Stand_Site[2]],
    [0,  -100,  Paras.Stand_Site[2]]
]

def coupled_oscillators(t, y):
    target_y = [0, np.pi, np.pi/2, 3*np.pi/2]
    if Paras.isStopping:
        return [0]*4
    omega = 0.2; K = 0.5
    dy = []
    for i in range(4):
        coupling = sum(np.sin(y[j]-y[i]-(target_y[i]-target_y[j])) for j in range(4))
        dy.append(2*np.pi*omega + K*coupling)
    return dy


def cpg_controller():
    """
    CPG 主控制器线程：生成相位，映射轨迹与角度，缓存 PWM 与足端 XYZ
    """
    start = time.perf_counter()
    # 初始化站立位——直接使用方格坐标
    Current_Sites = [pos.copy() for pos in initial_positions]
    # 平滑插补到初始坐标（省略实现）
    # ...
    time.sleep(1)

    # CPG 初始化参数
    y0 = [0.0, np.pi/2, np.pi, 3*np.pi/2]
    current_y = y0.copy()
    last_t = 0.0
    MappingSaws = [0, 3, 1, 2]

    # 循环 30 s
    while True:
        current_t = time.perf_counter() - start
        if current_t > 30:
            break
        t.append(current_t)

        for legid in range(4):
            # 归一化相位
            phase = current_y[MappingSaws[legid]] % (2*np.pi)
            sawSignals[legid].append(phase)
            CPGSignals[legid].append(current_y[MappingSaws[legid]])

            # 足端三维位置
            # 直接以 initial_positions 为基准
            base = initial_positions[legid]
            site = calculate_current_site(phase, legid)
            # site 相对基准，需要加 base 位移
            x = base[0] + (site[0] - Paras.Stand_Site[0])
            y = base[1] + (site[1] - Paras.Stand_Site[1])
            z = site[2]
            for coord, val in enumerate([x, y, z]):
                ppos_all[coord][legid].append(val)

            # 计算关节控制相位与逆运动学
            ctrl_phase = phase * Paras.controlSgn[legid] + Paras.controlPhase[legid]
            controlSignals[legid].append(ctrl_phase)
            joints = map_to_joints(ctrl_phase, legid)

            # 缓存 PWM 指令，记录角度
            for ji in range(3):
                control_angles[ji][legid] = get_joint_cmd(
                    legid, ji, joints[ji] * Paras.sgn[legid][ji]
                )
                angles[ji][legid].append(joints[ji])

        # 相位耦合与积分
        dy = coupled_oscillators(current_t, current_y)
        delta_t = current_t - last_t
        for i in range(4):
            current_y[i] += dy[i] * delta_t
        last_t = current_t

        time.sleep(0.01)


def send_command():
    """串口发送PWM指令线程"""
    while True:
        cmd_angles = []
        for legid in range(4):
            for ji in range(3):
                cmd_angles.append(control_angles[ji][legid])
        send_cmd(cmd_angles)
        time.sleep(0.01)


def plot_animation():
    """
    使用 FuncAnimation 绘制四足末端三维轨迹动图
    """
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')
    labels = ['左前','右前','右后','左后']
    lines = [ax.plot([], [], [], label=labels[i])[0] for i in range(4)]

    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')
    ax.set_xlim(min(min(ppos_all[0])), max(max(ppos_all[0])))
    ax.set_ylim(min(min(ppos_all[1])), max(max(ppos_all[1])))
    ax.set_zlim(min(min(ppos_all[2])), max(max(ppos_all[2])))
    ax.legend()
    plt.title('四足末端三维轨迹动图')

    def init():
        for line in lines:
            line.set_data([], [])
            line.set_3d_properties([])
        return lines

    def update(frame):
        for i, line in enumerate(lines):
            xs = ppos_all[0][i][:frame]
            ys = ppos_all[1][i][:frame]
            zs = ppos_all[2][i][:frame]
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
        return lines

    ani = FuncAnimation(fig, update, frames=len(t), init_func=init,
                        interval=50, blit=True)
    plt.show()

if __name__ == '__main__':
    threading.Thread(target=send_command, daemon=True).start()
    t1 = threading.Thread(target=cpg_controller)
    t1.start(); t1.join()

    # 播放动图
    plot_animation()
    print("Done")
