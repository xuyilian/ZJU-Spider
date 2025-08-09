import numpy as np
import matplotlib.pyplot as plt

# ---- 两个振荡器的耦合动力学 ----
def coupled_oscillators2(t, y, omega=0.6, K=1.0):
    target_y = [0.0, np.pi]  # 期望相差 π
    dy = []
    for i in range(2):
        j = 1 - i
        coupling = np.sin(y[j] - y[i] - (target_y[j] - target_y[i]))
        dy_i = 2 * np.pi * omega + K * coupling
        dy.append(dy_i)
    return dy


def coupled_oscillators_group6(t, y, omega=0.6, K=1.0):
    """
    六个振荡器分为两组，每组三个：同组期望相位差为0，异组为π，全耦合。
    """
    dy = []
    for i in range(6):
        coupling = 0
        for j in range(6):
            if i == j:
                continue
            same_group = (i // 3 == j // 3)
            delta = 0 if same_group else np.pi
            coupling += np.sin(y[j] - y[i] - delta)
        dy_i = 2 * np.pi * omega + K * coupling
        dy.append(dy_i)
    return dy

# ---- 六个振荡器的耦合动力学 ----
def coupled_oscillators6(t, y, omega=0.3, K=1.0):
    N = 6
    delta = np.pi / 3
    target_y = [i * delta for i in range(N)]
    dy = []
    for i in range(N):
        ip = (i + 1) % N
        im = (i - 1) % N
        coupling = (
            np.sin(y[ip] - y[i] - (target_y[ip] - target_y[i]))
            + np.sin(y[im] - y[i] - (target_y[im] - target_y[i]))
        )
        dy_i = 2 * np.pi * omega + K * coupling
        dy.append(dy_i)
    return dy


def coupled_oscillators_variable(t, y, omega=1.0, K=1.0, T=5.0):
    """
    六个振荡器，部分耦合对的目标相位差随时间指数变化。
    12,23,45,56: pi/3 -> 0
    16,34: pi/3 -> pi
    其它: 目标相位差恒为0
    """
    N = 6
    dy = [0] * N

    # 设定变化的比例因子（等比指数收敛，lambda控制变化快慢）
    lam = 3.0 / T   # 控制收敛速度
    w1 = np.exp(-lam * t)     # pi/3 -> 0
    w2 = 1 - np.exp(-lam * t) # 0 -> 1

    # 初始化所有目标相位差为0
    delta_matrix = np.zeros((N, N))

    # 12, 23, 45, 56: pi/3 等比变为0
    delta_matrix[0, 1] = delta_matrix[1, 0] = (np.pi/3) * w1
    delta_matrix[1, 2] = delta_matrix[2, 1] = (np.pi/3) * w1
    delta_matrix[3, 4] = delta_matrix[4, 3] = (np.pi/3) * w1
    delta_matrix[4, 5] = delta_matrix[5, 4] = (np.pi/3) * w1

    # 16, 34: pi/3 等比变为pi
    delta_matrix[0, 5] = delta_matrix[5, 0] = (np.pi/3)*(1-w2) + np.pi*w2
    delta_matrix[2, 3] = delta_matrix[3, 2] = (np.pi/3)*(1-w2) + np.pi*w2

    # 其余未指定的目标相位差设为0

    for i in range(N):
        coupling = 0
        # 只耦合相邻振荡器（1-2,2-3,3-4,4-5,5-6,6-1）
        for j in [(i+1)%N, (i-1)%N]:
            delta = delta_matrix[i, j]
            coupling += np.sin(y[j] - y[i] - delta)
        dy[i] = 2 * np.pi * omega + K * coupling
    return dy

def omega_schedule(t, T, omega0=0.3, omega1=0.6):
    """t: 当前时间, T: 总时长"""
    return omega0 if t < T/2 else omega1

def omega_linear(t, T, omega0=0.3, omega1=0.6):
    """
    omega 随时间 t 从 omega0 线性增长到 omega1
    t: 当前时间
    T: 总时长
    """
    if t >= T:
        return omega1
    return omega0 + (omega1 - omega0) * (t / T)


def coupled_oscillators_group6_variable(t, y, K=1.0, T=20.0, omega0=0.3, omega1=0.6):
    """
    12,23,45,56: 耦合目标相位差 pi/3，随omega从0.5到1等比变为0
    16,34: pi/3随omega从0.5到1等比变为pi
    其它为0
    """
    N = 6

    # omega 按时间阶梯变化
    omega = omega_linear(t, T, omega0, omega1)

    # 等比插值（指数映射，omega0->0, omega1->1）
    if omega1 == omega0:
        ratio = 0.0
    else:
        ratio = (omega - omega0) / (omega1 - omega0)
    # 等比过渡权重（类似于指数型，但此处用线性，如果你要指数映射可以替换）
    w1 = 1 - ratio  # pi/3 -> 0
    w2 = ratio      # 0   -> pi

    delta_matrix = np.zeros((N, N))

    # 12,23,45,56: pi/3->0
    for (a, b) in [(0, 1), (1, 2), (3, 4), (4, 5)]:
        delta_matrix[a, b] = delta_matrix[b, a] = (np.pi/3) * w1

    # 16,34: pi/3->pi
    for (a, b) in [(0, 5), (2, 3)]:
        delta_matrix[a, b] = delta_matrix[b, a] = (np.pi/3)*(1-w2) + np.pi*w2

    dy = []
    for i in range(N):
        coupling = 0
        # 只耦合与指定邻居
        for j in [ (i+1)%N, (i-1)%N ]:
            delta = delta_matrix[i, j]
            coupling += np.sin(y[j] - y[i] - delta)
        dy_i = 2 * np.pi * omega + K * coupling
        dy.append(dy_i)
    return dy


if __name__ == "__main__":
    # ==== 公共参数 ====
    omega = 0.5
    omega0 = 0.3
    omega1 = 0.6
    K = 1.0
    dt = 0.01
    T  = 10
    steps = int(T / dt)
    ts = np.linspace(0, T, steps)

    # === 六个振荡器 ===
    N = 6
    delta = np.pi / 3
    initial_phases = np.array([i * delta for i in range(N)])   # 初始相位正好相差pi/3
    y = initial_phases.copy()
    phi = np.zeros((N, steps))
    for k in range(steps):
        for i in range(N):
            phi[i, k] = y[i]
        dy = coupled_oscillators_group6_variable(ts[k], y, K=K, T=T, omega0=omega0, omega1=omega1)
        y = [y[i] + dy[i] * dt for i in range(N)]

    # 绘图：每条曲线颜色一致，0-pi 虚线、pi-2pi 实线
    plt.figure(figsize=(10,5))
    color_list = plt.cm.tab10(np.arange(N))
    for i in range(N):
       color = color_list[i % len(color_list)]
       plt.plot(ts, phi[i] % (2*np.pi), color=color, label=f'φ{i+1}')
    plt.xlabel('Time (s)')
    plt.ylabel('Phase (rad)')
    plt.title('Six Coupled Oscillators (initial phase matches target)')
    plt.legend()
    plt.grid(True)
    plt.show()