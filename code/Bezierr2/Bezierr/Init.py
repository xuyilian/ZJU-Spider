import math
import time

from Paras import Paras
import numpy as np
import serial.tools.list_ports
import time

def insec(r1, p2, r2):
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
        x2 =  A * a / d
        y2 =  A * b / d
        x3 = round(x2 - h * b / d, 2)
        y3 = round(y2 + h * a / d, 2)
        x4 = round(x2 + h * b / d, 2)
        y4 = round(y2 - h * a / d, 2)
        c1 = [x3, y3]
        c2 = [x4, y4]
        if y3 >= y4:
            return c1
        else:
            return c2

def get_angle_from_site(pos):
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
    print(inse)
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
    return [t1, t2, t3]


class STM32ServoController:
    def __init__(self, port='COM6', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate,
                                 timeout=1, write_timeout=1)
        time.sleep(2)  # 等待串口初始化
        print(f"已连接到 {port}，波特率 {baudrate}")

    def send_servo_angles(self, angles):
        """发送舵机角度数组"""
        if len(angles) != 12:  # 假设 SERVO_NUM=12
            raise ValueError("角度数组必须是12个值")

        angle_str = ','.join(map(str, angles)) + '\n'
        try:
            self.ser.write(angle_str.encode('utf-8'))
            print(f"已发送: {angle_str.strip()}")
        except serial.SerialTimeoutException:
            print("发送超时！")

    def read_feedback(self):
        """读取并解析STM32反馈"""
        feedback = []
        start_time = time.time()

        while time.time() - start_time < 1.0:  # 1秒超时
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:  # 有效数据行
                    if line == "":  # 遇到空行表示反馈结束
                        break
                    try:
                        servo_id, angle = map(int, line.split())
                        feedback.append((servo_id, angle))
                    except ValueError:
                        print(f"解析错误: {line}")

        return feedback

    def close(self):
        self.ser.close()
        print("串口已关闭")
