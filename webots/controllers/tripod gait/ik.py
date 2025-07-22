# ik.py

import numpy as np

# ——— 连杆长度（根据你的机器人参数修改） ———
L1 = 30.1   # Coxa 长度
L2 = 60.91  # Femur 长度
L3 = 152.32 # Tibia 长度

def circle_intersection(R, center, S):
    """
    计算半径为 R 的圆 (中心在原点) 与半径为 S 的圆 (中心在 center) 在平面上的交点，
    返回那个 y 较大的解点 (x,y)，若无交点则返回 None。
    """
    a, b = center
    d = np.hypot(a, b)
    if d > R + S or d < abs(R - S) or d == 0:
        return None
    A = (R**2 - S**2 + d**2) / (2 * d)
    h = np.sqrt(max(R**2 - A**2, 0))
    x2 = A * a / d
    y2 = A * b / d
    x3 = x2 - h * b / d
    y3 = y2 + h * a / d
    x4 = x2 + h * b / d
    y4 = y2 - h * a / d
    # 选择 y 较大的那个解
    return np.array([x3, y3]) if y3 > y4 else np.array([x4, y4])

def inverse_kinematics(target):
    """
    给定足端在机体坐标系下的目标位置 target = [x, y, z]（米）,
    计算 Coxa-Femur-Tibia 三关节的角度 (θ1, θ2, θ3)，单位为弧度。
    返回 (θ1, θ2, θ3)。若目标不可达则抛出 RuntimeError。
    """
    x, y, z = target
    # θ1：绕垂直轴的 Coxa 关节
    θ1 = np.arctan2(y, x)
    # 在 Coxa 平面内，投影到 Femur-Tibia 平面
    r = np.hypot(x, y) - L1
    # 计算 Femur-Tibia 圆相交，得到中间关节在平面内的坐标 j2 = (u, v)
    j2 = circle_intersection(L2, [r, z], L3)
    if j2 is None:
        raise RuntimeError(f"Target {target} unreachable")
    u, v = j2
    # θ2：Femur 相对于 Coxa 平面的抬起角（仰角）
    θ2 = np.arctan2(v, u)
    # 计算 Tibia 相对于 Femur 的角度 θ3
    # 向量从中间关节指向末端 [r-u, z-v]
    vec = np.array([r - u, z - v])
    α   = np.arctan2(vec[1], vec[0])
    θ3  = (α + np.pi/2) - θ2
    return θ1, θ2, θ3
