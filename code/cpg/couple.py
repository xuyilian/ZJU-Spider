import numpy as np
import matplotlib.pyplot as plt

# CPG 耦合振荡器参数
w = 0.2  # 基础频率 (Hz)
K = 0.2  # 耦合强度
target = np.array([0, np.pi/2, np.pi, 3*np.pi/2])  # 目标相位差

def coupled_oscillators(y):
    """计算四振荡器的相位变化率 dy/dt"""
    dy = []
    for i in range(4):
        coupling = np.sum(np.sin(y - y[i] - (target[i] - target)))
        dy_i = 2 * np.pi * w + K * coupling
        dy.append(dy_i)
    return np.array(dy)

# 仿真参数
dt = 0.01       # 时间步长 (s)
T = 20          # 总仿真时间 (s)
steps = int(T / dt)

# 初始相位（任意设置）
y = np.array([0.5, 1.5, 3.0, 5.0])

# 存储结果
ys = np.zeros((steps, 4))
ts = np.linspace(0, T, steps)

# 在 10s 处对振荡器 1 设置相位为 π/2
perturb_time = 10  # seconds
perturb_idx = int(perturb_time / dt)
new_phase = np.pi / 2  # 目标相位

# Euler 方法积分
for idx in range(steps):
    if idx == perturb_idx:
        y[1] = new_phase  # 将第二个振荡器（索引 1）相位设为 π/2
    dy = coupled_oscillators(y)
    y = y + dy * dt
    ys[idx] = np.mod(y, 2 * np.pi)

# 绘图
plt.figure()
for i in range(4):
    plt.plot(ts, ys[:, i], label=f'osc {i}')
plt.axvline(perturb_time, linestyle='--', color='gray', label='Perturbation')
plt.xlabel('Time (s)')
plt.ylabel('Phase (rad)')
plt.legend()
plt.title('Coupled Oscillator Phases with Reset to π/2')
plt.show()
