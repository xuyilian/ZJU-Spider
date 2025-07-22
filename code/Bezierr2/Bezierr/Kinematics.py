import math
import numpy as np
from Paras import Paras

def insec(r1, p2, r2):
    # print(r1, p2, r2, "----------")
    R = r1
    a = p2[0]
    b = p2[1]
    S = r2
    d = math.sqrt((abs(a)) ** 2 + (abs(b)) ** 2)
    if d > (R + S) or d < (abs(R - S)):
        # print("Two circles have no intersection")
        return None, None
    elif d == 0:
        # print("Two circles have same center!")
        return None, None
    else:
        A = (R ** 2 - S ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(R ** 2 - A ** 2)
        x2 = A * a / d
        y2 = A * b / d
        x3 = round(x2 - h * b / d, 2)
        y3 = round(y2 + h * a / d, 2)
        x4 = round(x2 + h * b / d, 2)
        y4 = round(y2 - h * a / d, 2)
        c1 = [x3, y3]
        c2 = [x4, y4]
        if y3 >= y4:
            return c1
        else:
            print("ERRORRRRR!!!!!!!!!!!!!!")
            return c2

def get_angle_from_site(pos):
    # 返回三个角的弧度制
    # 原控制直角位置为90 90 90， 此处返回为 0 0 0
    # calculate angle from site ref: --| outside - counterclockwise - clockwise
    x = pos[0]
    y = pos[1]
    z = pos[2]
    if y >= 0:
        t1 = math.atan2(y, x)
    else:
        t1 = - math.atan2(-y, x)
    x1 = math.sqrt(x ** 2 + y ** 2) - Paras.la
    z1 = z
    inse = insec(Paras.lb, [x1, z1], Paras.lc)
    t2 = math.atan2(inse[1], inse[0])
    inse_x = inse[0]
    inse_z = inse[1]
    if inse_x == x1:
        t3 = 0
        t3 = t2 + t3 + np.pi / 2
    elif inse_x > x1:
        t3 = math.atan2(inse_x - x1, inse_z - z1)
        t3 = t2 + t3 + np.pi / 2
    elif inse_z >= z1:
        t3 = math.atan2(x1 - inse_x, inse_z - z1)
        t3 = t2 - t3 + np.pi / 2
    else:
        t3 = math.atan2(z1 - inse_z, x1 - inse_x)
        t3 = t2 - t3
    t3 = t3 - np.pi / 2
    return np.array([t1, t2, t3])