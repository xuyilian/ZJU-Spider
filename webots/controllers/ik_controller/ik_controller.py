# ik_controller_three_legs_alljoints.py

from controller import Robot
import numpy as np
import Paras   # Paras.initial_offsets 中包含所有电机偏移

# ——— 连杆参数 ———
L1, L2, L3 = 30.1, 60.91, 152.32

# ——— 逆运动学函数 ———
def circle_intersection(R, center, S):
    a, b = center
    d = np.hypot(a, b)
    if d > R+S or d < abs(R-S) or d == 0:
        return None
    A = (R**2 - S**2 + d**2) / (2*d)
    h = np.sqrt(max(R**2 - A**2, 0))
    x2 = A*a/d; y2 = A*b/d
    x3 = x2 - h*b/d; y3 = y2 + h*a/d
    x4 = x2 + h*b/d; y4 = y2 - h*a/d
    return np.array([x3, y3]) if y3>y4 else np.array([x4, y4])

def inverse_kinematics(target):
    θ1 = np.arctan2(target[1], target[0])
    r  = np.hypot(target[0], target[1]) - L1
    z  = target[2]
    j2 = circle_intersection(L2, [r, z], L3)
    if j2 is None:
        raise RuntimeError("Target unreachable")
    θ2 = np.arctan2(j2[1], j2[0])
    vec = np.array([r - j2[0], z - j2[1]])
    α   = np.arctan2(vec[1], vec[0])
    θ3  = (α + np.pi/2) - θ2
    return θ1, θ2, θ3

# ——— 控制器主体 ———
robot    = Robot()
timestep = int(robot.getBasicTimeStep())

# 三条腿与各自目标点
legs = {
    'leg1': np.array([167.36 + 30*np.sqrt(3), 30.0, -89.84]),
    'leg2': np.array([167.36,                60.0, -89.84]),
    'leg3': np.array([167.36 - 30*np.sqrt(3), 30.0, -89.84])
}
sides = ['left','right']

# 构建电机和传感器名称
motor_names  = []
sensor_names = []
for leg in legs:
    for side in sides:
        for joint in ['coxa','femur','tibia']:
            motor_names.append(f"{leg}_{side}_{joint}")
            sensor_names.append(f"{leg}_{side}_{joint}_sensor")

motors  = [robot.getDevice(n) for n in motor_names]
sensors = [robot.getDevice(n) for n in sensor_names]
for m, s in zip(motors, sensors):
    m.setVelocity(1.0)
    s.enable(timestep)

# 读取初始偏移
offsets = [Paras.initial_offsets[name] for name in motor_names]

print("=== Applying IK to all 3 joints of leg1, leg2 and leg3 ===")
idx = 0
for leg, target in legs.items():
    θ1, θ2, θ3 = inverse_kinematics(target)
    # 左侧用负号，右侧用正号
    deltas = [-θ1, -θ2, -θ3,   θ1, θ2, θ3]  
    for side_i, side in enumerate(sides):
        for joint_i, joint in enumerate(['coxa','femur','tibia']):
            name  = f"{leg}_{side}_{joint}"
            m     = motors[idx]
            off   = offsets[idx]
            delta = deltas[side_i*3 + joint_i]
            cmd   = off + delta
            m.setPosition(cmd)
            print(f"{name:25s} offset={off:.4f} delta={delta:.4f} -> cmd={cmd:.4f}")
            idx += 1

# 让电机运动到位
for _ in range(100):
    if robot.step(timestep) == -1:
        break

# 打印实际读数验证
print("\n=== Actual joint angles ===")
for name, s in zip(motor_names, sensors):
    print(f"{name:25s} {s.getValue():.4f} rad")
