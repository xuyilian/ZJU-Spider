# init_femur_tibia_with_coxa.py

from controller import Robot
import Paras
import numpy as np

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    # 准备关节列表：包括 coxa、femur、tibia
    legs = [1, 2, 3]
    sides = ['left', 'right']
    coxa_joints  = [f'leg{leg}_{side}_coxa'  for leg in legs for side in sides]
    femur_joints = [f'leg{leg}_{side}_femur' for leg in legs for side in sides]
    tibia_joints = [f'leg{leg}_{side}_tibia' for leg in legs for side in sides]
    joint_names  = coxa_joints + femur_joints + tibia_joints

    # 获取 Motor 和 PositionSensor
    motors  = {}
    sensors = {}
    for name in joint_names:
        motors[name]  = robot.getDevice(name)
        sensors[name] = robot.getDevice(name + '_sensor')
        sensors[name].enable(ts)

    # 等一步，确保传感器有值
    if robot.step(ts) == -1:
        return

    # 读取初始偏移
    offsets = { name: Paras.initial_offsets[name] for name in joint_names }

    # 设置目标：coxa -> 0；femur/tibia 按 Paras.delta 和 Paras.delta_sign
    print("=== Setting targets ===")
    for name in joint_names:
        parts = name.split('_')  # ['legX','side','type']
        joint_type = parts[2]
        if joint_type == 'coxa':
            target = 0.0
            print(f"{name:20s} offset={offsets[name]:.4f} -> target={target:.4f} (reset)")
        else:
            side  = parts[1]
            delta = Paras.delta_changes[joint_type]
            sign  = Paras.delta_sign[joint_type][side]
            target = offsets[name] + sign * delta
            print(f"{name:20s} offset={offsets[name]:.4f} "
                  f"{'+' if sign>0 else '-'}{delta:.4f} -> target={target:.4f}")
        motors[name].setPosition(target)

    # 等若干步，让电机运动到位
    for _ in range(50):
        if robot.step(ts) == -1:
            break

    # 打印最终角度
    print("\n=== Final joint angles ===")
    for name in joint_names:
        actual = sensors[name].getValue()
        print(f"{name:<20s} {actual:.4f} rad")

if __name__ == "__main__":
    main()
