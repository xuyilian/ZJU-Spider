import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import Paras
# ——— 参数设置 ———
# 起点和终点示例 (可根据实际情况修改)
p1 = [167.36, 0, -89.84]
p3 = 0.2
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
        s_xy = (2*np.pi - theta) / np.pi

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

def ArcBezier(p1, p3, theta, z_lift, z_down):
    """
    p1, p3: 起点/终点三维坐标 [x,y,z]
    theta: 相位 [0,2π)
    z_lift, z_down: 抬腿/下压的高度修改值
    XY 平面走圆弧，Z 方向做 Bézier 曲线
    """
    # 转为 numpy
    p1 = np.array(p1, dtype=float)
    p3 = np.array(p3, dtype=float)
    # 中间点 Z 控制
    # --- 1) 计算圆弧半径 R ---
    side = Paras.side
    # 公式里的 cos(pi - |atan2|)
    R = side + p1[0]
    # --- 2) 圆弧上的参数 s_xy ---
    # 保证走完整个圆弧，theta∈[0,π] 对应 s∈[0,1]，其余退回
    if theta <= np.pi:
        s_xy = theta/np.pi
    else:
        s_xy = (2*np.pi - theta)/np.pi

    # XY 坐标就在该圆上
    x = R * np.cos((1/2 - s_xy)*p3) - side
    y = R * np.sin((1/2 - s_xy)*p3)

    # p2_z 在抬升/下压方向上的偏移
    if theta <= np.pi:
        s_z = theta/np.pi
        p2z_ctrl = p1[2] + z_lift
    else:
        s_z = (theta - np.pi)/np.pi
        p2z_ctrl = p1[2] + z_down

    def mix_axis(c0, c2, c3, s):
        return ((1 - s)**4 + 4*(1 - s)**3*s) * c0 + \
               6*(1 - s)**2 * s**2       * c2 + \
               (4*(1 - s)*s**3 + s**4)    * c3

    z = mix_axis(p1[2], p2z_ctrl, p1[2], s_z)
    # 注意：这里第二个参数实际上是对 z 方向 2 号控制点的权重，
    # 由于是三点四次曲线，代码把 p2 直接当成了 z1，z2 参数省略。

    return np.array([x, y, z])


if __name__ == "__main__":
 # ——— 采样轨迹 ———
   thetas = np.linspace(0, 2*np.pi, 200)
   trajectory = np.array([ArcBezier(p1, p3, th, z_lift, z_down) for th in thetas])

   
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
