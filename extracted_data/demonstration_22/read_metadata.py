import pickle
import numpy as np
import csv

META_PATH = "cam_0_rgb_video.metadata"
CSV_OUT = "cam_0_rgb_timestamps.csv"


def to_seconds(timestamps):
    """
    与你原代码完全一致：自动判断时间单位并转为秒
    """
    # 检查空数组
    if len(timestamps) == 0:
        print("  警告: 时间戳数组为空！")
        return timestamps

    sample_size = min(10, len(timestamps))
    val = np.mean(timestamps[:sample_size])

    # 检查是否为 NaN
    if np.isnan(val):
        print(f"  警告: 时间戳包含 NaN 值")
        return timestamps

    if val > 1e16:       # ns
        return timestamps / 1e9
    elif val > 1e13:     # us
        return timestamps / 1e6
    elif val > 1e11:     # ms
        return timestamps / 1e3
    else:                # s
        return timestamps


def load_camera_timestamps(meta_path):
    with open(meta_path, "rb") as f:
        meta = pickle.load(f)

    # 调试信息
    print(f"\n=== Metadata 结构诊断 ===")
    print(f"类型: {type(meta)}")

    if isinstance(meta, dict):
        print(f"字典键: {list(meta.keys())}")
        for k, v in meta.items():
            if isinstance(v, (list, np.ndarray)):
                print(f"  - {k}: {type(v)}, 长度={len(v)}")
            else:
                print(f"  - {k}: {type(v)}, 值={v}")
    elif isinstance(meta, list):
        print(f"列表长度: {len(meta)}")
        if len(meta) > 0:
            print(f"第一个元素类型: {type(meta[0])}")
    print("=" * 30 + "\n")

    if isinstance(meta, dict) and "timestamps" in meta:
        ts = np.array(meta["timestamps"])
    elif isinstance(meta, list):
        ts = np.array(meta)
    else:
        ts = None
        if isinstance(meta, dict):
            for k in meta:
                if "time" in k.lower():
                    ts = np.array(meta[k])
                    break
        if ts is None:
            raise ValueError("无法识别 metadata 中的时间戳结构")

    print(f"提取的时间戳数量: {len(ts)}")
    if len(ts) > 0:
        print(f"前 3 个值: {ts[:3]}")
        print(f"后 3 个值: {ts[-3:]}")

    return to_seconds(ts)


def main():
    ts_sec = load_camera_timestamps(META_PATH)

    with open(CSV_OUT, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_index", "timestamp_sec"])

        for i, t in enumerate(ts_sec):
            writer.writerow([i, t])

    print(f"已导出 {len(ts_sec)} 条时间戳 → {CSV_OUT}")


if __name__ == "__main__":
    main()
