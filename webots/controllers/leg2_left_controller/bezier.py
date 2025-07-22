import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ——— 参数设置 ———
# 起点和终点示例 (可根据实际情况修改)
p1 = [167.36-30*np.sqrt(3), -30.0, -89.84]
p3 = [167.36+30*np.sqrt(3),  30.0,  -89.84]
# 抬腿和下压高度
z_lift = 10
z_down = -2

def Bezier(p1, p3, theta, z_lift, z_down):
    p1 = np.array(p1, dtype=float)
    p3 = np.array(p3, dtype=float)
    p2 = (p1 + p3) * 0.5

    # XY阶段参数
    if theta <= np.pi:
        s_xy = theta / np.pi
    else:
        s_xy = (theta - np.pi) / np.pi

    # Z阶段参数
    if theta <= np.pi:
        s_z = theta / np.pi
        p2[2] += z_lift
    else:
        s_z = (theta - np.pi) / np.pi
        p2[2] += z_down

    def mix_axis(c0, c2, c3, s):
        return ((1 - s)**4 + 4*(1 - s)**3*s) * c0 + \
               6*(1 - s)**2 * s**2       * c2 + \
               (4*(1 - s)*s**3 + s**4)    * c3

    x = mix_axis(p1[0], p2[0], p3[0], s_xy)
    y = mix_axis(p1[1], p2[1], p3[1], s_xy)
    z = mix_axis(p1[2], p2[2], p3[2], s_z)
    return np.array([x, y, z])

# ——— 采样轨迹 ———
thetas = np.linspace(0, 2*np.pi, 200)
trajectory = np.array([Bezier(p1, p3, th, z_lift, z_down) for th in thetas])

# ——— 绘制 3D 轨迹 ———
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], linewidth=2)
ax.scatter(*p1, s=50, marker='o')
ax.text(*p1, 'p1')
ax.scatter(*p3, s=50, marker='o')
ax.text(*p3, 'p3')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Bezier Trajectory')
plt.show()

# ——— 绘制 Z 随相位的变化 ———
plt.figure()
plt.plot(thetas, trajectory[:,2])
plt.xlabel('theta (rad)')
plt.ylabel('Z')
plt.title('Z component vs. theta')
plt.grid(True)
plt.show()
