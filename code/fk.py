import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Link lengths
L1 = 30.1    # Coxa length
L2 = 60.91   # Femur length
L3 = 152.32  # Tibia length

# Sample joint angles (in radians)
theta1 = np.deg2rad(0)    # yaw of Coxa
theta2 = np.deg2rad(-35)    # 0 means femur horizontal
theta3 = np.deg2rad(0)    # 0 means tibia vertical down

# Rotation matrices
def Rz(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,               0,            1]
    ])

def Ry(theta):
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [ 0,             1, 0            ],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

# Forward kinematics with new baseline:
# - femur at theta2=0 -> horizontal (along local x)
# - tibia at theta3=0 -> vertical down (local -z)
# transform:
#   J0 at origin
#   J1 = Rz(theta1) @ [L1,0,0]
#   J2 = J1 + Rz(theta1)@Ry(theta2)@[L2,0,0]
#   EE = J2 + Rz(theta1)@Ry(theta2)@Ry(theta3)@[0,0,-L3]
j0 = np.zeros(3)
j1 = Rz(theta1) @ np.array([L1,0,0])
j2 = j1 + Rz(theta1) @ Ry(theta2) @ np.array([L2,0,0])
ee = j2 + Rz(theta1) @ Ry(theta2) @ Ry(theta3) @ np.array([0,0,-L3])

print(f"End effector position: x={ee[0]:.2f}, y={ee[1]:.2f}, z={ee[2]:.2f}")
# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Links
ax.plot(*zip(j0,j1), color='k', lw=3, label='Coxa')
ax.plot(*zip(j1,j2), color='b', lw=3, label='Femur')
ax.plot(*zip(j2,ee), color='g', lw=3, label='Tibia')
# Points
for name,pt,c in [('J1',j0,'k'),('J2',j1,'b'),('J3',j2,'g'),('EE',ee,'m')]:
    ax.scatter(*pt, color=c, s=50)
    ax.text(*(pt+[2,2,2]), name)
ax.set_xlabel('X');ax.set_ylabel('Y');ax.set_zlabel('Z')
ax.set_title('FK with femur horizontal, tibia down at zero')
ax.legend()
plt.show()
