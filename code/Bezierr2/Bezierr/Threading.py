#!/usr/bin/python
# -*- coding: UTF-8 -*-

import threading
import time

from Init import insec, get_angle_from_site, STM32ServoController
from BezierMapping import map_to_joints, calculate_current_site
import numpy as np
import matplotlib.pyplot as plt
from STM32CommunicateInt import send_cmd, get_joint_cmd
from Paras import Paras
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 线程同步与停止信号
control_lock = threading.Lock()
data_lock = threading.Lock()
stop_event   = threading.Event()

# 全局数据
t               = []                                 # 时间序列
angles          = [[[], [], [], []] for _ in range(3)]  # 3 个关节 × 4 条腿
sawSignals      = [[] for _ in range(4)]            # 锯齿波信号
controlSignals  = [[] for _ in range(4)]            # 控制相位信号
control_angles  = [[1500] * 4 for _ in range(3)]    # 3×4 PWM 目标
ppos_all        = [[[] for _ in range(4)] for _ in range(3)]  # 足端位置

def get_leg_index(legid):
    """腿索引转换，1 和 3 号腿顺序反过来"""
    return 4 - legid if legid in (1, 3) else legid

def getSite(initialSite, flg, legid):
    """根据步态标志生成当前支撑/摆动末端位置"""
    x, y, z = initialSite
    return [x, y + flg * Paras.y_step, z]

def sawtooth(theta):
    """将相位转换为范围 [0,1] 的锯齿波"""
    return (theta % (2 * np.pi)) / (2 * np.pi)

def coupled_oscillators(t, y):
    """四个振荡器的耦合动力学方程"""
    if Paras.isStopping:
        return [0] * 4

    omega = 0.3
    K     = 0.5
    target_y =  [0, np.pi, np.pi/2, 3*np.pi/2]
    dy = []
    for i in range(4):
        coupling = sum(
            np.sin(y[j] - y[i] - (target_y[i] - target_y[j]))
            for j in range(4)
        )
        dy.append(2 * np.pi * omega + K * coupling)
    return dy

def cpg_controller():
    """中枢模式发生器控制主循环"""
    global t, angles, sawSignals, controlSignals, control_angles, ppos_all

    # 初始站立位置：初始PWM 与目标PWM
    Current_Sites = [Paras.Default_Site] * 4
    Sites = [getSite(Paras.Stand_Site, -3, 0),getSite(Paras.Stand_Site, 1, 1),getSite(Paras.Stand_Site, 1, 2),getSite(Paras.Stand_Site, -3, 3)]

    # 上电后插值到初始位置
    current_pwm = []
    angles1     = []
    for legid in range(4):
        targ_angles = get_angle_from_site(Sites[legid])
        cur_angles  = get_angle_from_site(Current_Sites[legid])
        for joint in range(3):
            ag1 = targ_angles[joint] * Paras.sgn[legid][joint] / np.pi * 180 + 90
            ag2 = cur_angles[joint] * Paras.sgn[legid][joint] / np.pi * 180 + 90
            angles1.append(int(500 + ag1 * (2000/180)))
            current_pwm.append(int(500 + ag2 * (2000/180)))

    # 平滑插值到初始位置
    is_ok = False
    while not is_ok:
        is_ok = True
        for idx in range(len(current_pwm)):
            if current_pwm[idx] != angles1[idx]:
                is_ok = False
                current_pwm[idx] += 1 if current_pwm[idx] < angles1[idx] else -1
                leg = idx // 3
                joint = idx % 3
                with control_lock:
                    control_angles[joint][leg] = current_pwm[idx]
        time.sleep(0.015)

    time.sleep(5)  # 等待机械臂就绪

    # 振荡器初始相位
    y0 = [0.0, np.pi, 3*np.pi/2, np.pi/2]
    start_time = time.perf_counter()
    last_t     = 0.0
    current_y  = y0.copy()

    # 主循环：步态持续 30 秒
    while True:
        if stop_event.is_set():
            break
        current_t = time.perf_counter() - start_time
        if current_t > 30:
            break

        # 记录时间
        with data_lock:
            t.append(current_t)

        # 每条腿计算
        for legid in range(4):
            phase = current_y[legid] % (2 * np.pi)
            saw   = sawtooth(phase)
            control_phase = phase * Paras.controlSgn[legid] + Paras.controlPhase[legid]
            site = calculate_current_site(phase, legid)

            # 保存末端位置
            with data_lock:
                for coord in range(3):
                    ppos_all[coord][legid].append(site[coord])

            # 关节映射与指令转换
            joints = map_to_joints(control_phase, legid)
            idx = legid
            with control_lock:
                control_angles[0][idx] = get_joint_cmd(legid, 0, joints[0] * Paras.sgn[idx][0])
                control_angles[1][idx] = get_joint_cmd(legid, 1, joints[1] * Paras.sgn[idx][1])
                control_angles[2][idx] = get_joint_cmd(legid, 2, joints[2] * Paras.sgn[idx][2])

            # 存储信号用于绘图
            with data_lock:
                sawSignals[legid].append(saw)
                controlSignals[legid].append(control_phase)
                for j in range(3):
                    angles[j][legid].append(joints[j])

        # 更新相位
        dy = coupled_oscillators(current_t, current_y)
        delta_t = current_t - last_t
        last_t = current_t
        for i in range(4):
            current_y[i] += delta_t * dy[i]

        time.sleep(0.005)

def send_command():
    """持续发送 PWM 指令到 STM32"""
    while not stop_event.is_set():
        with control_lock:
            cmds = []
            for legid in range(4):
                idx = get_leg_index(legid)
                for joint in range(3):
                    cmds.append(control_angles[joint][idx])
        try:
            send_cmd(cmds)
        except Exception as e:
            print(f"[Error] send_cmd failed: {e}")
        time.sleep(0.005)

def plot_result():
    """绘制相位、控制信号与关节角度"""
    plt.rcParams['font.sans-serif'] = ['SimSun']
    plt.rcParams['axes.unicode_minus'] = False

    # 锯齿波
    plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.plot(t, sawSignals[i], label=f'θ{i} 锯齿波', linestyle='--')
    plt.xlabel('时间 (s)')
    plt.ylabel('归一化相位')
    plt.legend()
    plt.grid(True)
    plt.show()

    # 控制相位
    plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.plot(t, controlSignals[i], label=f'腿{i} 控制相位', linestyle='--')
    plt.xlabel('时间 (s)')
    plt.ylabel('相位 (rad)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # 三个关节角度
    joint_names = ['髋关节', '膝关节', '踝关节']
    for j in range(3):
        plt.figure(figsize=(10, 6))
        for i in range(4):
            plt.plot(t, angles[j][i], label=f'腿{i} {joint_names[j]}')
        plt.xlabel('时间 (s)')
        plt.ylabel(f'{joint_names[j]} 角度 (rad)')
        plt.legend()
        plt.grid(True)
        plt.show()

    for legid in range(4):
        plt.figure(figsize=(10, 6))
        plt.plot(t, ppos_all[0][legid], label='x')
        plt.plot(t, ppos_all[1][legid], label='y')
        plt.plot(t, ppos_all[2][legid], label='z')
        plt.xlabel('时间 (s)')
        plt.ylabel('位置 (m)')
        plt.title(f'腿{legid} 末端位置轨迹')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # 启动线程
    t1 = threading.Thread(target=cpg_controller)
    t2 = threading.Thread(target=send_command, daemon=True)

    t2.start()
    time.sleep(1)   # 确保 send_command 先启动
    t1.start()
    t1.join()       # 等待 CPG 控制完成

    # 停止发送线程并等待退出
    stop_event.set()
    t2.join()

    # 绘制并完成
    plot_result()
    print("Done")

