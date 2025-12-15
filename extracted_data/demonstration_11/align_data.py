import h5py
import numpy as np
import pickle
import os

# === 配置 ===
FILE_PATHS = {
    "camera_meta": "cam_0_rgb_video.metadata", 
    "left_arm_joint": "left_franka_joint_states.h5",
    "left_arm_cart": "left_franka_cartesian_states.h5",
    "right_arm_joint": "right_franka_joint_states.h5",
    "right_arm_cart": "right_franka_cartesian_states.h5",
    "left_gripper": "left tesollo_joint_states.h5",
    "right_gripper": "right tesollo_joint_states.h5"
}

OUTPUT_FILE = "aligned_data.h5"

def to_seconds(timestamps):
    """
    自动检测时间单位并统一转换为秒 (Seconds)。
    基于当前Unix时间戳的数量级判断。
    """
    # 取第一个非零值或均值来判断
    val = np.mean(timestamps[:10])
    
    # 阈值判断 (以1973年作为分界线)
    if val > 1e16: 
        # 纳秒 (Nanoseconds) -> 除以 1e9
        return timestamps / 1e9
    elif val > 1e13: 
        # 微秒 (Microseconds) -> 除以 1e6
        return timestamps / 1e6
    elif val > 1e11: 
        # 毫秒 (Milliseconds) -> 除以 1e3
        return timestamps / 1e3
    else:
        # 秒 (Seconds)
        return timestamps

def load_camera_timestamps(meta_path):
    print(f"正在加载相机时间轴: {meta_path}")
    with open(meta_path, 'rb') as f:
        meta = pickle.load(f)
    
    if isinstance(meta, dict) and 'timestamps' in meta:
        ts = np.array(meta['timestamps'])
    elif isinstance(meta, list):
        ts = np.array(meta)
    else:
        # 尝试寻找包含 time 的 key
        ts = None
        if isinstance(meta, dict):
            for k in meta.keys():
                if 'time' in k:
                    ts = np.array(meta[k])
                    break
        if ts is None:
            raise ValueError(f"无法识别 metadata 结构: {type(meta)}")

    # 统一转为秒
    return to_seconds(ts)

def find_nearest_indices(target_times, source_times):
    # 确保 source_times 有序
    idx_sorted = np.searchsorted(source_times, target_times)
    idx_sorted = np.clip(idx_sorted, 0, len(source_times) - 1)
    
    final_indices = idx_sorted.copy()
    
    for i, idx in enumerate(idx_sorted):
        if idx > 0:
            diff_curr = abs(source_times[idx] - target_times[i])
            diff_prev = abs(source_times[idx-1] - target_times[i])
            if diff_prev < diff_curr:
                final_indices[i] = idx - 1
    return final_indices

def process_h5_file(h5_path, cam_times_sec, output_grp, prefix_name):
    if not os.path.exists(h5_path):
        print(f"警告: 文件不存在 {h5_path}，跳过。")
        return

    print(f"正在处理: {prefix_name} ({h5_path})...")
    
    with h5py.File(h5_path, 'r') as f:
        # 1. 寻找时间戳 Key
        # 你的数据里可能是 'timestamps'
        ts_key = 'timestamps'
        if ts_key not in f.keys():
            print(f"  错误: 找不到 {ts_key}。跳过。")
            return
        
        # 2. 读取并转换时间单位
        source_times = f[ts_key][:]
        source_times_sec = to_seconds(source_times)
        
        # 3. 对齐计算
        indices = find_nearest_indices(cam_times_sec, source_times_sec)
        
        # 检查对齐质量
        aligned_times = source_times_sec[indices]
        diffs = np.abs(aligned_times - cam_times_sec)
        max_diff = np.max(diffs)
        mean_diff = np.mean(diffs)
        
        print(f"  - 原始单位量级: {source_times[0]:.1e} (已转为秒)")
        print(f"  - 平均同步误差: {mean_diff*1000:.2f} ms")
        print(f"  - 最大同步误差: {max_diff*1000:.2f} ms")
        
        if mean_diff > 0.05: # 50ms
             print("  [严重警告] 时间戳似乎未对齐，请检查是否为相对时间 vs 绝对时间。")

        # 4. 保存数据
        # 创建子组
        grp = output_grp.create_group(prefix_name)
        
        for key in f.keys():
            # 跳过时间戳本身（如果你想存对齐后的，可以在下面手动存）
            if key == ts_key:
                continue

            # === 修复 ValueError: 跳过标量和非 Dataset 对象 ===
            obj = f[key]
            if not isinstance(obj, h5py.Dataset):
                continue
            if obj.shape == (): # 标量判断
                # print(f"  - 跳过标量 metadata: {key}")
                continue
            
            # 读取数据
            data = obj[:]
            
            # 只有当第一维长度等于源时间戳长度时，才进行对齐切片
            if data.shape[0] == len(source_times):
                aligned_data = data[indices]
                grp.create_dataset(key, data=aligned_data, compression="gzip")
            else:
                # 长度不匹配的数据（可能是其他配置信息），直接复制或忽略
                # print(f"  - 跳过非时间序列数据: {key} {data.shape}")
                pass
                
        # 保存对齐后的时间戳方便Debug
        grp.create_dataset('timestamps', data=aligned_times)

def main():
    try:
        cam_times_sec = load_camera_timestamps(FILE_PATHS["camera_meta"])
    except Exception as e:
        print(f"无法读取相机metadata: {e}")
        return

    with h5py.File(OUTPUT_FILE, 'w') as out_f:
        # 保存相机基准时间
        out_f.create_dataset("camera_timestamps", data=cam_times_sec)
        
        process_h5_file(FILE_PATHS["left_arm_joint"], cam_times_sec, out_f, "left_arm_joint")
        process_h5_file(FILE_PATHS["left_arm_cart"], cam_times_sec, out_f, "left_arm_cart")
        process_h5_file(FILE_PATHS["right_arm_joint"], cam_times_sec, out_f, "right_arm_joint")
        process_h5_file(FILE_PATHS["right_arm_cart"], cam_times_sec, out_f, "right_arm_cart")
        process_h5_file(FILE_PATHS["left_gripper"], cam_times_sec, out_f, "left_gripper")
        process_h5_file(FILE_PATHS["right_gripper"], cam_times_sec, out_f, "right_gripper")

    print(f"\n=== 全部完成 ===")
    print(f"文件已保存至: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()