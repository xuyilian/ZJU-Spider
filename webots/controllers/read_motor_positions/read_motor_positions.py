# read_all_joint_positions_once.py

from controller import Robot, Motor, PositionSensor

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # 构造名称列表
    legs = [1, 2, 3]
    sides = ['left', 'right']
    parts = ['coxa', 'femur', 'tibia']

    motor_sensor_pairs = []
    for i in legs:
        for side in sides:
            for part in parts:
                motor_name = f'leg{i}_{side}_{part}'
                sensor_name = motor_name + '_sensor'
                try:
                    motor = robot.getDevice(motor_name)
                except:
                    continue
                sensor = None
                try:
                    sensor = robot.getDevice(sensor_name)
                    sensor.enable(timestep)
                except:
                    pass
                motor_sensor_pairs.append((motor, sensor))

    # 运行一个时间步以获取有效传感器读数
    robot.step(timestep)

    # 仅读取并打印一次
    print("=== Joint positions (single read) ===")
    for motor, sensor in motor_sensor_pairs:
        if sensor:
            angle = sensor.getValue()
            print(f"{motor.getName():<20} {angle:.4f} rad")
        else:
            print(f"{motor.getName():<20} no sensor")

    # 结束控制器（让仿真保持运行）
    # robot.step(-1)  # 不再循环

if __name__ == '__main__':
    main()
