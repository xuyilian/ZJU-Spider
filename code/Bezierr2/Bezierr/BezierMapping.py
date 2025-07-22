import numpy as np
from Paras import Paras
from Kinematics import get_angle_from_site


def Bezier(p1, p3, theta, legid=None):
    """
    通用的 4 次 Bezier 插值，支持摆动抬腿和特殊的 stance 下压处理。
    对 Z 轴走特殊分段逻辑；对 X/Y 轴仅在摆动相与支撑相间隔断。
    参数：p1 起点；p3 终点；theta 相位 [0,2π)；legid 腿编号。
    """
    p1 = np.array(p1, dtype=float)
    p3 = np.array(p3, dtype=float)
    # 控制点中点
    p2 = (p1 + p3) * 0.5

    # --- 计算用于 X/Y 方向的插值参数 s_xy ---
    # 仅根据摆动（0→π/2）和支撑（π/2→2π）两段
    if theta <= np.pi/2:
        s_xy = theta * 2 / np.pi
    else:
       s_xy =(theta - np.pi/2) / (2 * np.pi - np.pi/2)


    # --- 计算用于 Z 方向的插值参数 s_z 与高度调整 ---
    if 0 <= theta <= np.pi/2:
        # 摆动相抬腿
        s_z = theta * 2 / np.pi
        p2[2] += Paras.z_lift
    else:
        # 支撑相及特殊下压区间
        if legid in (0, 1) and np.pi/2 < theta <= np.pi:
            # 腿 0,1 在 π/2→π 下压
            p2[2] += Paras.z_down
            s_z = (theta - np.pi/2) / (np.pi/2)
        elif legid in (2, 3) and 3*np.pi/2 < theta < 2*np.pi:
            # 腿 2,3 在 3π/2→2π 下压
            p2[2] += Paras.z_down
            s_z = (theta - 3*np.pi/2) / (np.pi/2)
        else:
            # 其它支撑相平滑过渡
            s_z = 1.0

    # --- 分轴合成 Bezier ---
    # 对 X,Y 轴使用 s_xy；对 Z 轴使用 s_z
    def mix_axis(coord0, coord2, coord3, s):
        return ((1 - s)**4 + 4*(1 - s)**3*s) * coord0 + \
               6*(1 - s)**2 * s**2 * coord2 + \
               (4*(1 - s)*s**3 + s**4) * coord3

    # 分量插值
    x = mix_axis(p1[0], p2[0], p3[0], s_xy)
    y = mix_axis(p1[1], p2[1], p3[1], s_xy)
    z = mix_axis(p1[2], p2[2], p3[2], s_z)

    return np.array([x, y, z])


def angle_from_bezier(p1, p3, theta, legid):
    """Bezier → 逆运动学角度，若 legid > 1 则返回角度取反"""
    angles = get_angle_from_site(Bezier(p1, p3, theta, legid))
    #if legid > 1:
        # 对后两条腿取反，以保持对称运动
       # return [-a for a in angles]
    return angles


def calculate_current_site(theta, legid):
    """
    根据 theta, legid 计算当前足端三维位置，支持下压阶段
    """
    initial_site = Paras.InitialSite[legid]
    flg = 1 if legid <= 1 else -1
    final_site = [
        initial_site[0],
        initial_site[1] + 6 * flg * Paras.y_step,
        initial_site[2]
    ]
    if theta<=np.pi/2:
        # 传入 legid 支持 Bezier 中的 special-case
        return Bezier(initial_site, final_site, theta, legid)
    else:
        # 传入 legid 支持 Bezier 中的 special-case
        return Bezier(final_site, initial_site, theta, legid)


def map_to_joints(theta, legid):
    """
    将控制相位 theta 转为 3 个关节角度，支持 stop/delay 及 special-case
    """
    flg = 1 if legid <= 1 else -1

    # 正常摆动 & 支撑
    initial_site = Paras.InitialSite[legid]
    final_site = [
        initial_site[0],
        initial_site[1] + 6 * flg * Paras.y_step,
        initial_site[2]
    ]
    # controlSgn = +1: 正常前摆/后收
    if Paras.controlSgn[legid] == 1:
        if theta <= np.pi / 2:
            # swing: initial → final
            return angle_from_bezier(initial_site, final_site, theta, legid)
        else:
            # stance: final → initial
            return angle_from_bezier(final_site, initial_site, theta, legid)

