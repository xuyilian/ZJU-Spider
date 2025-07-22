import numpy as np

class Paras:
    ratio = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    ratio = ratio/np.sum(ratio)

    balance_flag = -1
    balance_offset_sign = [[1, -1], [-1, -1], [-1, 1], [1, 1]]
    balance_offset_value = 10
    # 如果迈第i只腿时不平衡，重心需要相对于原来的移动balance_offset_sign[i] * balance_offset_value

    # 常数
    eps = 1e-3
    inf = 1e3

    # 机构参数，中心到四只腿基座的距离(x,y)
    lena = 36
    lenb = 36

    # 机构参数，腿的长度
    la = 27.5
    lb = 55.0
    lc = 97.5

    # 触地状态判断
    isOnGround = [1, 1, 1, 1]
    shouldOnGround = [1, 1, 1, 1]
    withSensor = [1, 1, 1, 1]
    adjustState = [0, 0, 0, 0] # 1表示提前触地，-1表示延后触地，0表示摆动
    adjustSite = [[],[],[],[]]  # 进入提前或延后触地时的位置
    adjustPhase = [0, 0, 0, 0]
    controlSgn = [1, 1, 1, 1]
    controlPhase = [0, 0, 0, 0]
    isStopping = 0

    delaySgn = [0, 0, 0, 0]
    delayPhase = [0,0,0,0]
    delaySite = [[],[],[],[]]
    delay_vz = 1/(np.pi/2)


    # 位置参数： 各个状态位置
    x_default = 82.5
    y_default = 0
    z_default = -97.5
    x_stand = 80.0
    y_stand = 40.0
    z_stand = -50.0
    z_stand1 = -50
    x_up = 90
    y_up = 20
    z_up = -10
    z_lift = 100
    z_down = 10
    y_step = 15
    Default_Site = [x_default, y_default, z_default]
    Stand_Site = [x_stand, y_stand, z_stand]
    Forward1_Site = [x_stand, y_stand + 1 * y_step, z_stand]
    Back1_Site = [x_stand, y_stand - y_step, z_stand]
    Mid_up_Site = [x_up, y_up, z_up]

    # 位置参数，根部坐标系到重心坐标系
    relativePara = [[-1, 1, 1, -1], [1, 1, -1, -1]]


    # CPG轨迹
    def getSite(self, flg, legid):
        x = self.x_stand
        y = self.y_stand
        z = self.z_stand
        if legid >= 2:
            z = self.z_stand1
        return [x, y + flg * Paras.y_step, z]

    InitialSite = [[x_stand, y_stand - 3 * y_step, z_stand], [x_stand, y_stand - 3 *  y_step, z_stand],
                   [x_stand, y_stand + 3 * y_step, z_stand1], [x_stand, y_stand + 3 * y_step, z_stand1]]
    # 时间参数
    t_legup = 3
    t_adjust = 3

    # 代码参数
    # SQP算法迈腿序号
    order_of_step = [2, 1, 3, 0]

    # 控制参数，实际角度与计算角度的符号差别
    sgn = [[-1, 1, 1], [1, -1, -1], [-1, 1, 1], [1, -1, -1]]
