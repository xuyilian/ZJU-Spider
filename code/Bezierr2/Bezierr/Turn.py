import numpy as np
from Paras import Paras

def compute_rotation_coord(InitialSiteSet, dx, dy, dtheta):  #
    alpha = -dtheta
    trans_matrix = np.array([[np.cos(alpha), np.sin(alpha), dx], [-np.sin(alpha), np.cos(alpha), dy], [0, 0, 1]])
    res = []
    for legid in range(4): # 计算第legid只脚转弯后在初始髋关节坐标系下的位置
        hip_coord = [InitialSiteSet[legid][0], InitialSiteSet[legid][1], InitialSiteSet[legid][2]] # 当前脚在髋关节坐标系下的位置，默认xy都为正
        # 当前脚在重心坐标系下的位置
        com_coord = [(hip_coord[0] + Paras.lena) * Paras.relativePara[0][legid], (hip_coord[1] +\
                     Paras.lenb) * Paras.relativePara[1][legid], hip_coord[2]]
        las_xy = np.array([[com_coord[0]], [com_coord[1]], [1]])
        new_xy = np.dot(trans_matrix, las_xy)
        com_coord_new = [new_xy[0][0], new_xy[1][0], com_coord[2]]

        hip_coord_new = [com_coord_new[0] * Paras.relativePara[0][legid] - Paras.lena, com_coord_new[1] *\
                         Paras.relativePara[1][legid] - Paras.lenb, com_coord_new[2]]
        res.append(hip_coord_new)
    return res

if __name__ == "__main__":
    init_site = [Paras.Stand_Site ]* 4
    print(init_site)
    res = compute_rotation_coord(init_site, 0, 0, np.pi / 12)
    print(res)
