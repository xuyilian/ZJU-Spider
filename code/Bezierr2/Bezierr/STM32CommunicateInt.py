import serial
import serial.tools.list_ports
import struct  # 用于打包二进制数据
import numpy as np

# 列出可用串口
ports_list = list(serial.tools.list_ports.comports())
if len(ports_list) <= 0:
    print("无串口设备。")
else:
    print("可用的串口设备如下：")
    for comport in ports_list:
        print(list(comport)[0], list(comport)[1])

# 初始化串口
ser = serial.Serial(port="COM5",
                        timeout=0.005,
                        baudrate=460800)

def get_joint_cmd(legid, jointid, angle):  # 第几只脚， 第几个关节， 弧度制多少度
    angle = angle + np.pi / 2
    angle = angle / np.pi * 180
    ag = int(500 + (2000 / 180) * angle)
    return ag

def send_cmd(angles):
    if len(angles) != 12:
        raise ValueError("角度数组必须是12个值")

    angles_int = [int(angle) for angle in angles]  # 转为整数
    checksum = sum(angles_int) & 0xFFFF  # 计算校验和

    # 打包数据：头(0xAA 0xBB) + 12个uint16_t + 校验和（大端）
    data = struct.pack('>BB12HH',
                       0xAA, 0xBB,  # 头
                       *angles_int,  # 12个角度
                       checksum)  # 校验和
    ser.write(data)
    # print(f"Sent: {angles_int}")
