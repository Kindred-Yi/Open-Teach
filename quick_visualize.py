#!/usr/bin/env python3
"""
快速可视化 aligned_data.h5 - 简化版
直接运行即可生成可视化图表

使用方法:
    python quick_visualize.py
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt

# ===== 配置区域 =====
H5_FILE = 'extracted_data/demonstration_11/aligned_data.h5'
OUTPUT_FILE = 'visualization_output.png'
TIME_RANGE = None  # 设置为 [10, 20] 可以只显示 10-20 秒的数据
# ===================

def main():
    print("=" * 70)
    print(f"读取文件: {H5_FILE}")
    print("=" * 70)

    with h5py.File(H5_FILE, 'r') as f:
        # 1. 打印文件结构
        print("\n【文件结构】")
        for group_name in f.keys():
            item = f[group_name]

            # 检查是 Dataset 还是 Group
            if isinstance(item, h5py.Dataset):
                print(f"\nDataset: {group_name}")
                print(f"  Shape: {item.shape}")
            elif isinstance(item, h5py.Group):
                print(f"\nGroup: {group_name}/")
                for key in item.keys():
                    if isinstance(item[key], h5py.Dataset):
                        print(f"  - {key}: {item[key].shape}")

        # 2. 提取数据
        print("\n【提取数据】")
        data_to_plot = []
        timestamps = None

        for group_name in f.keys():
            item = f[group_name]

            # 处理时间戳
            if 'timestamp' in group_name.lower() and timestamps is None:
                timestamps = item[:]
                print(f"找到时间戳: {group_name}, {len(timestamps)} 个数据点")
                continue

            # 如果是 Group，遍历其内容
            if isinstance(item, h5py.Group):
                # 获取时间戳
                if 'timestamps' in item and timestamps is None:
                    timestamps = item['timestamps'][:]
                    print(f"找到时间戳: {len(timestamps)} 个数据点")

                # 获取关节位置数据
                for pos_key in ['joint_positions', 'positions', 'joint_position']:
                    if pos_key in item:
                        positions = item[pos_key][:]
                        data_to_plot.append({
                            'name': group_name,
                            'positions': positions,
                            'num_joints': positions.shape[1] if len(positions.shape) > 1 else 1
                        })
                        print(f"  {group_name}: {positions.shape[1]} 个关节")
                        break

            # 如果是 Dataset，检查是否是关节数据
            elif isinstance(item, h5py.Dataset):
                if 'position' in group_name.lower() or 'joint' in group_name.lower():
                    positions = item[:]
                    if len(positions.shape) >= 2:  # 确保是二维数据
                        data_to_plot.append({
                            'name': group_name,
                            'positions': positions,
                            'num_joints': positions.shape[1] if len(positions.shape) > 1 else 1
                        })
                        print(f"  {group_name}: {positions.shape}")

        if timestamps is None:
            print("错误: 找不到时间戳!")
            return

        if not data_to_plot:
            print("错误: 找不到关节数据!")
            return

        # 3. 转换为相对时间
        relative_time = timestamps - timestamps[0]

        # 4. 时间范围过滤
        if TIME_RANGE:
            mask = (relative_time >= TIME_RANGE[0]) & (relative_time <= TIME_RANGE[1])
            relative_time = relative_time[mask]
            for item in data_to_plot:
                item['positions'] = item['positions'][mask]

        # 5. 创建可视化
        print(f"\n【生成可视化】")
        print(f"时间范围: {relative_time[0]:.2f}s ~ {relative_time[-1]:.2f}s")
        print(f"绘制 {len(data_to_plot)} 个组件")

        num_components = len(data_to_plot)
        fig, axes = plt.subplots(num_components, 1, figsize=(14, 4 * num_components))

        # 如果只有一个组件，axes 不是数组
        if num_components == 1:
            axes = [axes]

        for idx, item in enumerate(data_to_plot):
            ax = axes[idx]
            name = item['name']
            positions = item['positions']
            num_joints = item['num_joints']

            # 绘制每个关节
            for joint_idx in range(num_joints):
                ax.plot(relative_time, positions[:, joint_idx],
                       label=f'Joint {joint_idx}',
                       alpha=0.8,
                       linewidth=1.5)

            ax.set_xlabel('Time (seconds)', fontsize=12)
            ax.set_ylabel('Joint Position (radians)', fontsize=12)
            ax.set_title(f'{name} - {num_joints} Joints', fontsize=14, fontweight='bold')
            ax.legend(loc='upper right', fontsize=9, ncol=min(4, num_joints))
            ax.grid(True, alpha=0.3)

        plt.suptitle('Robot Joint Trajectories Over Time',
                    fontsize=16, fontweight='bold', y=0.995)
        plt.tight_layout()

        # 6. 保存
        plt.savefig(OUTPUT_FILE, dpi=150, bbox_inches='tight')
        print(f"\n✓ 图表已保存到: {OUTPUT_FILE}")

        # 7. 数据统计
        print("\n【数据统计】")
        for item in data_to_plot:
            positions = item['positions']
            print(f"\n{item['name']}:")
            print(f"  数据点数: {len(positions)}")
            print(f"  关节数: {item['num_joints']}")
            print(f"  位置范围: [{np.min(positions):.3f}, {np.max(positions):.3f}]")
            print(f"  平均值: {np.mean(positions):.3f}")
            print(f"  标准差: {np.std(positions):.3f}")

if __name__ == '__main__':
    main()
