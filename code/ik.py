import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ——— 参数定义 ———
L1 = 30.1    # Coxa 长度
L2 = 60.91   # Femur 长度
L3 = 152.32  # Tibia 长度

# 目标末端位置 (x, y, z)
target = np.array([167.36 + 30*np.sqrt(3), 30.0, -89.84])

# ——— 平面圆相交函数 ———
def circle_intersection(R, center, S):
    a, b = center
    d = np.hypot(a, b)
    if d > R + S or d < abs(R - S) or d == 0:
        return None
    A = (R**2 - S**2 + d**2) / (2*d)
    h = np.sqrt(max(R**2 - A**2, 0))
    x2 = A * a / d; y2 = A * b / d
    x3 = x2 - h * b / d; y3 = y2 + h * a / d
    x4 = x2 + h * b / d; y4 = y2 - h * a / d
    return np.array([x3, y3]) if y3 > y4 else np.array([x4, y4])

# ——— 1) 求 θ₁ ———
theta1 = np.arctan2(target[1], target[0])

# ——— 2) 计算平面内交点 joint2_2d ———
r = np.hypot(target[0], target[1]) - L1
z = target[2]
joint2_2d = circle_intersection(L2, [r, z], L3)
if joint2_2d is None:
    raise ValueError("目标超出可达范围！")

# ——— 3) 求 θ₂ ———
theta2 = np.arctan2(joint2_2d[1], joint2_2d[0])

# ——— 4) 求 θ₃，令 “Tibia 垂直向下”时 θ₃=0 ———
#    在平面内 Tibia 链节向量：
tibia_vec = np.array([r - joint2_2d[0], z - joint2_2d[1]])
#    该向量相对于水平线的角度：
alpha = np.arctan2(tibia_vec[1], tibia_vec[0])
#    当 Tibia 垂直向下时，相对于水平的角度是 -90° (-π/2)。
#    我们定义 θ₃ = (alpha - (-π/2)) - θ₂
theta3 = (alpha + np.pi/2) - theta2

# ——— 打印三关节角度（度） ———
print(f"θ₁ (Coxa yaw)   = {np.rad2deg(theta1):.2f}°")
print(f"θ₂ (Femur pitch)= {np.rad2deg(theta2):.2f}°")
print(f"θ₃ (Tibia fold) = {np.rad2deg(theta3):.2f}°")

# ——— 计算各关节 3D 坐标 ———
joint0 = np.zeros(3)
joint1 = np.array([L1*np.cos(theta1), L1*np.sin(theta1), 0.0])
joint2 = np.array([
    (L1 + joint2_2d[0]) * np.cos(theta1),
    (L1 + joint2_2d[0]) * np.sin(theta1),
    joint2_2d[1]
])
end_eff = target

# ——— 可视化 ———
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制三段连杆
for (p, q, col, name) in [
    (joint0,  joint1, 'k', 'Coxa'),
    (joint1,  joint2, 'b', 'Femur'),
    (joint2,  end_eff, 'g', 'Tibia')
]:
    ax.plot(*zip(p, q), color=col, lw=3, label=name)

# 标注关节和末端
labels = {
    'Joint 1\nθ₁': joint0,
    'Joint 2\nθ₂': joint1,
    'Joint 3\nθ₃': joint2,
    'End effector': end_eff
}
for name, pt in labels.items():
    ax.scatter(*pt, s=80, color='m' if 'End' in name else 'k')
    ax.text(*(pt + np.array([2,2,2])), name)

# 自动缩放坐标系
pts = np.vstack([joint0, joint1, joint2, end_eff])
min_pt = pts.min(axis=0) - 20
max_pt = pts.max(axis=0) + 20
ax.set_xlim(min_pt[0], max_pt[0])
ax.set_ylim(min_pt[1], max_pt[1])
ax.set_zlim(min_pt[2], max_pt[2])

# 设置
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D IK with Tibia Vertical=θ₃=0')
ax.legend()
plt.show()
