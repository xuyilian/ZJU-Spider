import numpy as np
from Paras import Paras
from Kinematics import get_angle_from_site

def Bezier(control_points, theta): # 给出控制点和相位，获得贝塞尔曲线
    p1 = np.array(control_points[0])
    p3 = np.array(control_points[2])
    p2 = np.array(control_points[1])
    p0 = p1
    p4 = p3
    if theta <= np.pi / 2:
        s = theta * 2 / np.pi
        p2[2] += Paras.z_lift
    else:
        s = (theta * 2 /  np.pi - 1) / 3
    p_current = ((1 - s) ** 4 + 4 * (1 - s) ** 3 * s) * p0 +\
                6 * (1 - s) ** 2 * s ** 2 * p2 +\
                (4 * (1 - s) * s ** 3 + s ** 4) * p3
    return p_current

def angle_from_bezier(p1, p3, theta):
    return get_angle_from_site(Bezier(p1, p3, theta))


def calculate_current_site(theta, legid):
    initial_site = Paras.InitialSite[legid]
    if legid <= 1:
        flg = 1
    else:
        flg = -1
    final_site = [initial_site[0], initial_site[1] + 6 * flg * Paras.y_step, initial_site[2]]
    return Bezier([initial_site, initial_site, final_site], theta)


def map_to_joints(theta, legid): # 返回初始位置为initial_site的腿在第theta相位时的位置
    flg = -1
    if legid <= 1:
        flg = 1

    if Paras.isStopping == 1 and Paras.delaySgn[legid] == 1: # 还没触地的脚
        Paras.delaySite[legid][2] -= Paras.delay_vz
        current_site = Paras.delaySite[legid]
        Paras.InitialSite[legid] = [current_site[0], current_site[1] - flg * 6 * Paras.y_step, current_site[2]]
        return get_angle_from_site(current_site)
    else:
        initial_site = Paras.InitialSite[legid]
        final_site = [initial_site[0], initial_site[1] + 6 * flg * Paras.y_step, initial_site[2]]
        if Paras.controlSgn[legid] == 1:
            if theta <= np.pi / 2:
                return angle_from_bezier(initial_site, final_site, theta)
            else:
                return angle_from_bezier(final_site, initial_site, theta)
        else: # control = -1
            theta = -theta
            return []