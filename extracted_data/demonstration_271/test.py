import h5py
import matplotlib.pyplot as plt

# 读取 h5 文件
with h5py.File("right_franka_cartesian_states.h5", "r") as f:
    positions = f["positions"][:]     # shape (2035, 7)
    timestamps = f["timestamps"][:]   # shape (2035,)

# 绘制每个关节的轨迹
plt.figure(figsize=(10, 6))
for joint_idx in range(positions.shape[1]):
    plt.plot(timestamps, positions[:, joint_idx], label=f"Joint {joint_idx+1}")

plt.xlabel("Time (s)")
plt.ylabel("Joint Position (rad)")
plt.title("Franka Joint Positions Over Time")
plt.legend()
plt.grid(True)
plt.show()
