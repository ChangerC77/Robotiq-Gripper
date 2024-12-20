import numpy as np
from scipy.spatial.transform import Rotation as R

"""
读取到的机械臂末端pose, 在机械臂和夹爪联合抓取时,此时要把这里的pose看作是gripper->hand,然后去算夹爪代替了末端pose之后,末端pose的位姿
"""

# hot dog
# pose = [0.6128, -0.059, -0.235, 1.023, 2.970, 0]
# orange 
pose = [0.4088, 0.3503, -0.2201, -1.2022, 2.9024, 0]
# 旋转向量转旋转矩阵
r_v = R.from_rotvec(np.array(pose)[-3:]).as_matrix()

# 齐次矩阵
gripper2base = np.zeros((4, 4))
gripper2base[:3, :3] = r_v
gripper2base[:3, 3] = np.array(pose)[:3]
gripper2base[3, 3] = 1 
# print("gripper->base的齐次矩阵")
# print(gripper2base)

# gripper -> hand z方向+0.19m
gripper2hand = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0.19],
                         [0, 0, 0, 1]])
# gripper -> base = hand ->base * gripper -> hand
# hand->base = gripper->base * hand->gripper(即为gripper->hand的转置)
# print("hand->base的齐次变换")
hand2base = np.dot(gripper2base, np.linalg.inv(gripper2hand))

# 齐次矩阵转6dof pose
x, y, z = hand2base[:3, 3]
hand2base_rm = hand2base[:3, :3]
r = R.from_matrix(hand2base_rm)
rotation_vector = R.as_rotvec(r)
rx, ry, rz = rotation_vector[:]
hand2base = [x, y, z, rx, ry, rz]
print(hand2base) 