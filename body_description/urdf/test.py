# ik_with_extra_joint_fixed_api.py

import numpy as np
import matplotlib.pyplot as plt
from math import pi
from ikpy.chain import Chain
from ikpy.link import DHLink

def main():
    # 1. 从 URDF 加载原始链
    urdf_path = r"C:/Users/bj/iCloudDrive/Desktop/windows file/research/spider/body_description/urdf/body_description.urdf"
    base_chain = Chain.from_urdf_file(urdf_path)

    # 2. 找到 link2（joint2 后面那个 link）的索引
    link2_name = "leg1_left_leg_joint2"
    idx_link2 = next(i for i, link in enumerate(base_chain.links) if link.name == link2_name)

    # 3. 重建 link 列表，在 idx_link2 之后插入 DHLink（0.1 m 长，绕 Z 轴可转）
    new_links = []
    for i, link in enumerate(base_chain.links):
        new_links.append(link)
        if i == idx_link2:
            # DHLink 构造函数签名 (d, a, alpha, theta, bounds)
            extra = DHLink(
                0.1,    # d = 0.1 m
                0.0,    # a = 0.0
                0.0,    # alpha = 0.0
                0.0,    # theta offset = 0
                (-pi, pi)  # bounds
            )
            extra.name = "ExtraJoint"  # 手动给它一个好记的名字
            new_links.append(extra)

    # 4. 用新的 link 列表构建 Chain
    extended_chain = Chain(name="extended", links=new_links)

    # 5. 定义目标位姿并求解 IK
    target_frame = np.eye(4)
    target_frame[:3, 3] = [0.2, -0.1, 0.0]
    joint_angles = extended_chain.inverse_kinematics_frame(
        target_frame,
        initial_position=[0.0] * len(extended_chain.links)
    )

    # 6. 打印每个 joint 的名字和角度
    print("Computed joint angles (rad):")
    for link, angle in zip(extended_chain.links, joint_angles):
        name = getattr(link, "name", link.__class__.__name__)
        print(f"  {name:15s}: {angle:.4f}")

    # 7. 计算全部连杆的世界齐次变换
    frames = extended_chain.forward_kinematics(joint_angles, full_kinematics=True)

    # 8. 可视化：画出骨架、目标，以及每个 joint 名称
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    extended_chain.plot(joints=joint_angles, ax=ax, target=target_frame, show=False)

    for i, link in enumerate(extended_chain.links):
        x, y, z = frames[i][:3, 3]
        name = getattr(link, "name", f"link{i}")
        ax.text(x, y, z, name, size=8, color='black')

    # 9. 突出 ExtraJoint 那一段连杆
    p2 = frames[idx_link2][:3, 3]
    p_extra = frames[idx_link2+1][:3, 3]
    ax.scatter(*p2,      color='orange', s=50, label=link2_name)
    ax.scatter(*p_extra, color='green',  s=50, label="ExtraJoint end")
    ax.plot(
        [p2[0], p_extra[0]],
        [p2[1], p_extra[1]],
        [p2[2], p_extra[2]],
        '--', linewidth=2, label="Extra segment"
    )
    mid = (p2 + p_extra) / 2
    ax.text(*mid, '0.10 m', color='blue', size=8)

    # 10. 最后美化并展示
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('IK with Extra Joint after link2')
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
