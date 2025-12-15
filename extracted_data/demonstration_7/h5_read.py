import h5py

h5_file = "cam_0_depth.h5"  # 换成你的 h5 文件名

with h5py.File(h5_file, "r") as f:
    print("Top-level keys:", list(f.keys()))
    for k in f.keys():
        dset = f[k]
        print(f"{k}: shape={dset.shape}, dtype={dset.dtype}")
        if k == "timestamps":
            print("timestamps[0:5] =", dset[:5])  # 打印前5个时间戳
