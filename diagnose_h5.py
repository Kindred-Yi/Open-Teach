#!/usr/bin/env python3
"""诊断 h5 文件结构"""

import h5py
import numpy as np

h5_file = 'extracted_data/demonstration_11/aligned_data.h5'

print("=" * 70)
print(f"诊断文件: {h5_file}")
print("=" * 70)

with h5py.File(h5_file, 'r') as f:

    print("\n【顶层键】")
    print(f"Keys: {list(f.keys())}")

    print("\n【详细结构】")

    def show_structure(name, obj, level=0):
        indent = "  " * level
        if isinstance(obj, h5py.Dataset):
            data = obj[:]
            print(f"{indent}📄 Dataset: {name}")
            print(f"{indent}   Shape: {obj.shape}")
            print(f"{indent}   Dtype: {obj.dtype}")
            if np.issubdtype(obj.dtype, np.number) and data.size > 0:
                print(f"{indent}   Range: [{np.min(data):.3f}, {np.max(data):.3f}]")
            if data.size > 0 and data.size <= 5:
                print(f"{indent}   Values: {data}")
            print()
        elif isinstance(obj, h5py.Group):
            print(f"{indent}📁 Group: {name}/")
            for key in obj.keys():
                show_structure(key, obj[key], level + 1)

    for key in f.keys():
        show_structure(key, f[key], 0)

    print("\n【数据提取建议】")

    # 查找时间戳
    timestamp_keys = []
    for key in f.keys():
        if 'timestamp' in key.lower():
            timestamp_keys.append(key)

    if timestamp_keys:
        print(f"✓ 找到时间戳: {timestamp_keys}")
        print(f"  使用方法: timestamps = f['{timestamp_keys[0]}'][:]")

    # 查找关节数据
    position_keys = []
    for key in f.keys():
        item = f[key]
        if isinstance(item, h5py.Dataset):
            if 'position' in key.lower() or 'joint' in key.lower():
                if len(item.shape) >= 2:
                    position_keys.append((key, item.shape))

    if position_keys:
        print(f"\n✓ 找到关节位置数据:")
        for key, shape in position_keys:
            print(f"  - {key}: {shape}")
            print(f"    使用方法: positions = f['{key}'][:]")

    print("\n【完整访问路径】")

    def print_paths(name, obj, path=""):
        current_path = f"{path}/{name}" if path else name
        if isinstance(obj, h5py.Dataset):
            print(f"  f['{current_path}'][:] -> shape={obj.shape}")
        elif isinstance(obj, h5py.Group):
            for key in obj.keys():
                print_paths(key, obj[key], current_path)

    for key in f.keys():
        print_paths(key, f[key])

print("\n" + "=" * 70)
