import h5py
import matplotlib.pyplot as plt

# 读取 h5 文件
with h5py.File("right_franka_joint_states.h5", "r") as f:
    positions = f["positions"][:]     # shape (2035, 7)
    timestamps = f["timestamps"][:]   # shape (2035,)

# 绘制每个关节的轨迹
print(positions.shape)
