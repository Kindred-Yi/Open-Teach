#!/usr/bin/env python3
"""
交互式探索 h5 文件数据
提供多种可视化选项

使用方法:
    python explore_h5_data.py --file extracted_data/demonstration_11/aligned_data.h5
"""

import argparse
import h5py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Slider


def explore_h5_file(filepath):
    """交互式探索 h5 文件"""

    with h5py.File(filepath, 'r') as f:
        print("\n" + "=" * 70)
        print(f"文件: {filepath}")
        print("=" * 70)

        # 递归打印结构
        def print_attrs(name, obj):
            indent = "  " * name.count('/')
            if isinstance(obj, h5py.Group):
                print(f"{indent}📁 {name}/")
                if obj.attrs:
                    for key, value in obj.attrs.items():
                        print(f"{indent}    @{key}: {value}")
            elif isinstance(obj, h5py.Dataset):
                print(f"{indent}📄 {name}")
                print(f"{indent}    Shape: {obj.shape}")
                print(f"{indent}    Dtype: {obj.dtype}")
                if obj.size > 0:
                    if np.issubdtype(obj.dtype, np.number):
                        data = obj[:]
                        print(f"{indent}    Range: [{np.min(data):.3f}, {np.max(data):.3f}]")
                        print(f"{indent}    Mean: {np.mean(data):.3f}")

        print("\n【完整文件结构】")
        f.visititems(print_attrs)


def plot_interactive(filepath):
    """创建交互式绘图"""

    with h5py.File(filepath, 'r') as f:
        # 收集所有可绘制的数据
        plottable_data = {}
        timestamps = None

        for group_name in f.keys():
            group = f[group_name]

            # 查找时间戳
            if 'timestamps' in group:
                timestamps = group['timestamps'][:]

            # 查找关节数据
            for key in group.keys():
                if 'position' in key.lower() or 'joint' in key.lower():
                    dataset = group[key]
                    if isinstance(dataset, h5py.Dataset) and len(dataset.shape) > 0:
                        plottable_data[f"{group_name}/{key}"] = dataset[:]

        if timestamps is None:
            print("错误: 找不到时间戳数据")
            return

        if not plottable_data:
            print("错误: 找不到可绘制的数据")
            return

        # 转换为相对时间
        relative_time = timestamps - timestamps[0]

        # 创建图形
        fig, ax = plt.subplots(figsize=(14, 8))
        plt.subplots_adjust(left=0.3, bottom=0.15)

        # 初始绘图
        lines = {}
        for name, data in plottable_data.items():
            if len(data.shape) > 1:
                # 多关节数据，绘制第一个关节
                line, = ax.plot(relative_time, data[:, 0],
                               label=f"{name} [Joint 0]",
                               alpha=0.7, linewidth=2)
                lines[name] = {'line': line, 'data': data, 'joint_idx': 0}
            else:
                # 单维数据
                line, = ax.plot(relative_time, data,
                               label=name,
                               alpha=0.7, linewidth=2)
                lines[name] = {'line': line, 'data': data, 'joint_idx': None}

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Value', fontsize=12)
        ax.set_title('Interactive Joint Data Viewer', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)

        plt.show()


def compare_components(filepath, component_names=None):
    """对比多个组件的数据"""

    with h5py.File(filepath, 'r') as f:
        timestamps = None

        # 获取时间戳
        for group_name in f.keys():
            if 'timestamps' in f[group_name]:
                timestamps = f[group_name]['timestamps'][:]
                break

        if timestamps is None:
            print("错误: 找不到时间戳")
            return

        relative_time = timestamps - timestamps[0]

        # 如果没有指定组件，使用所有组件
        if component_names is None:
            component_names = list(f.keys())

        # 创建对比图
        fig, axes = plt.subplots(len(component_names), 1, figsize=(12, 4*len(component_names)))
        if len(component_names) == 1:
            axes = [axes]

        for idx, comp_name in enumerate(component_names):
            if comp_name not in f:
                continue

            group = f[comp_name]
            ax = axes[idx]

            # 查找位置数据
            positions = None
            for key in ['joint_positions', 'positions', 'joint_position']:
                if key in group:
                    positions = group[key][:]
                    break

            if positions is None:
                ax.text(0.5, 0.5, f'{comp_name}: No position data',
                       ha='center', va='center', transform=ax.transAxes)
                continue

            # 绘制所有关节
            if len(positions.shape) > 1:
                num_joints = positions.shape[1]
                for joint_idx in range(num_joints):
                    ax.plot(relative_time, positions[:, joint_idx],
                           label=f'Joint {joint_idx}', alpha=0.7)
            else:
                ax.plot(relative_time, positions, label='Position')

            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.set_title(f'{comp_name}', fontweight='bold')
            ax.legend(loc='upper right', ncol=3, fontsize=8)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('component_comparison.png', dpi=150, bbox_inches='tight')
        print("对比图已保存到: component_comparison.png")
        plt.show()


def export_to_csv(filepath, output_dir='exported_data'):
    """导出数据为 CSV 文件"""
    import os
    import pandas as pd

    os.makedirs(output_dir, exist_ok=True)

    with h5py.File(filepath, 'r') as f:
        for group_name in f.keys():
            group = f[group_name]

            for dataset_name in group.keys():
                dataset = group[dataset_name]
                if not isinstance(dataset, h5py.Dataset):
                    continue

                data = dataset[:]

                # 创建 DataFrame
                if len(data.shape) == 1:
                    df = pd.DataFrame({dataset_name: data})
                elif len(data.shape) == 2:
                    columns = [f'{dataset_name}_{i}' for i in range(data.shape[1])]
                    df = pd.DataFrame(data, columns=columns)
                else:
                    print(f"跳过高维数据: {group_name}/{dataset_name}")
                    continue

                # 保存 CSV
                csv_path = os.path.join(output_dir, f'{group_name}_{dataset_name}.csv')
                df.to_csv(csv_path, index=False)
                print(f"导出: {csv_path}")

    print(f"\n所有数据已导出到: {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description='探索和可视化 h5 文件数据')
    parser.add_argument('--file', required=True, help='h5 文件路径')
    parser.add_argument('--action', choices=['explore', 'plot', 'compare', 'export'],
                       default='explore', help='操作类型')
    parser.add_argument('--components', nargs='+', help='要对比的组件名称')
    parser.add_argument('--output-dir', default='exported_data', help='CSV 导出目录')

    args = parser.parse_args()

    if args.action == 'explore':
        explore_h5_file(args.file)

    elif args.action == 'plot':
        plot_interactive(args.file)

    elif args.action == 'compare':
        compare_components(args.file, args.components)

    elif args.action == 'export':
        export_to_csv(args.file, args.output_dir)


if __name__ == '__main__':
    main()
