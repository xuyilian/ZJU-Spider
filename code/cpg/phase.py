import numpy as np
import matplotlib.pyplot as plt

# 参数设置
N = 4                # 振荡器数量
T = 50.0             # 仿真总时长
dt = 0.01            # 时间步长
time = np.arange(0, T, dt)

# 固有频率 (rad/s)
omega = np.array([1.0, 1.0, 1.0, 1.0])

# 耦合权重矩阵，非对角元素设为 0.5
W = np.full((N, N), 0.5)
np.fill_diagonal(W, 0.5)

# 耦合相位 φ_i 分别为 0, π/2, π, 3π/2
phi = np.array([0, np.pi/2, np.pi, 3*np.pi/2])

# 初始化相位 (随机)
theta = np.zeros((len(time), N))
theta[0] = np.random.rand(N) * 2 * np.pi

# 欧拉法积分
for k in range(len(time) - 1):
    th = theta[k]
    dth = omega.copy()
    for i in range(N):
        dth[i] += np.sum(W[i] * np.sin(th - th[i] - phi[i]))
    theta[k + 1] = th + dth * dt

# 将相位限制在 [0, 2π)
theta_mod = np.mod(theta, 2*np.pi)

# 绘图
plt.figure()
for i in range(N):
    plt.plot(time, theta_mod[:, i], label=f'振荡器 {i+1}')
plt.xlabel('time (s)')
plt.ylabel('phase (rad)')
plt.title(' 0-2π')
plt.yticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
           ['0', 'π/2', 'π', '3π/2', '2π'])
plt.legend()
plt.show()
