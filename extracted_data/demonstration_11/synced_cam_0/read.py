import h5py
import numpy as np

sync_file = "synced_data.h5"

with h5py.File(sync_file, "r") as f:
    cam_ts = f["camera/timestamps"][:]              # shape (N,)，单位秒
    rel_time = f["camera/relative_time"][:]
    print("RGB frames:", cam_ts.shape[0])
    print("duration:", cam_ts[-1] - cam_ts[0], "seconds")

    # 访问某个机器人通道（数据集名称 = 原 h5 文件名）
    left_joint = f["robot/left_franka_joint_states/positions"][:]   # shape (N, dof)
    right_joint = f["robot/right_franka_joint_states/positions"][:]

    # 确认长度与相机一致
    assert left_joint.shape[0] == cam_ts.shape[0]
    assert right_joint.shape[0] == cam_ts.shape[0]

    # 简单检查：相邻帧的时间增量
    dt = np.diff(cam_ts)
    print("mean dt:", dt.mean(), "std:", dt.std())

    # 示例：取第 123 帧的所有数据
    idx = 123
    print("time:", cam_ts[idx], "left joints:", left_joint[idx], "right joints:", right_joint[idx])
