from controller import Robot, InertialUnit

def main():
    # 创建 Robot 实例
    robot = Robot()
    # 获取基本时间步长
    timestep = int(robot.getBasicTimeStep())

    # 获取并使能 InertialUnit 设备，假设其名称为 "imu"
    imu = robot.getDevice("imu")
    imu.enable(timestep)

    # 先运行一个时间步以确保传感器初始化
    robot.step(timestep)

    # 读取姿态数据：roll, pitch, yaw（单位：弧度）
    roll, pitch, yaw = imu.getRollPitchYaw()
    print(f"IMU orientation:\n  Roll:  {roll:.4f} rad\n  Pitch: {pitch:.4f} rad\n  Yaw:   {yaw:.4f} rad")

    # 如果需要四元数，可以使用下面方法：
    # q0, q1, q2, q3 = imu.getQuaternion()
    # print(f"IMU quaternion:\n  [{q0:.4f}, {q1:.4f}, {q2:.4f}, {q3:.4f}]")

    # 控制器运行结束
    # robot.step(-1)

if __name__ == "__main__":
    main()
