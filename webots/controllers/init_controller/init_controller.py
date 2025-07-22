# init_controller.py

from controller import Robot
import Paras

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # 获取所有电机设备
    motors = [robot.getDevice(name) for name in Paras.motor_names]

    # 初始化为位置控制模式，设定一个安全速度
    for m in motors:
        m.setPosition(float('inf'))
        m.setVelocity(1.0)

    # 等一步，让设备就绪
    if robot.step(timestep) == -1:
        return

    # 下发初始偏移角
    for name, m in zip(Paras.motor_names, motors):
        offset = Paras.initial_offsets.get(name, 0.0)
        m.setPosition(offset)

    # 运行一段，让腿部运动到位
    for _ in range(100):
        if robot.step(timestep) == -1:
            break

if __name__ == "__main__":
    main()
