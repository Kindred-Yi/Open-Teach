#!/usr/bin/env python3
"""
可视化 aligned_data.h5 中的机器人关节数据
显示各个组件的关节值随时间变化的曲线

使用方法:
    python visualize_aligned_data.py --demo 11
    python visualize_aligned_data.py --file extracted_data/demonstration_11/aligned_data.h5
"""

import argparse
import os
import h5py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def parse_args():
    parser = argparse.ArgumentParser(description='可视化对齐后的机器人数据')
    parser.add_argument('--demo', type=int, help='演示编号 (例如 11)')
    parser.add_argument('--file', type=str, help='直接指定 h5 文件路径')
    parser.add_argument('--storage-path', default='extracted_data', help='数据存储根目录')
    parser.add_argument('--output', default='aligned_data_visualization.png', help='输出图片路径')
    parser.add_argument('--dpi', type=int, default=150, help='图片分辨率')
    parser.add_argument('--time-range', nargs=2, type=float, help='时间范围 [开始秒 结束秒]')
    return parser.parse_args()


def inspect_h5_structure(filepath):
    """探索 h5 文件结构"""
    print("=" * 70)
    print(f"读取文件: {filepath}")
    print("=" * 70)

    structure = {}
    with h5py.File(filepath, 'r') as f:
        print("\n【文件结构】")

        for group_name in f.keys():
            print(f"\n组: {group_name}/")
            structure[group_name] = {}

            if isinstance(f[group_name], h5py.Group):
                for dataset_name in f[group_name].keys():
                    dataset = f[group_name][dataset_name]
                    if isinstance(dataset, h5py.Dataset):
                        print(f"  - {dataset_name}: shape={dataset.shape}, dtype={dataset.dtype}")
                        structure[group_name][dataset_name] = {
                            'shape': dataset.shape,
                            'dtype': dataset.dtype
                        }

    return structure


def load_aligned_data(filepath):
    """加载对齐后的数据"""
    data = {}

    with h5py.File(filepath, 'r') as f:
        # 读取所有组
        for group_name in f.keys():
            group = f[group_name]
            data[group_name] = {}

            # 读取组内所有数据集
            for dataset_name in group.keys():
                dataset = group[dataset_name]
                if isinstance(dataset, h5py.Dataset):
                    data[group_name][dataset_name] = dataset[:]

            # 读取组的属性
            data[group_name]['_attrs'] = dict(group.attrs)

    return data


def plot_joint_trajectories(data, output_path='aligned_data_visualization.png',
                           dpi=150, time_range=None):
    """
    绘制所有组件的关节轨迹

    Args:
        data: 从 h5 文件加载的数据字典
        output_path: 输出图片路径
        dpi: 图片分辨率
        time_range: 可选的时间范围 [start_sec, end_sec]
    """

    # 提取时间戳
    timestamps = None
    for group_name in data.keys():
        if 'timestamps' in data[group_name]:
            timestamps = data[group_name]['timestamps']
            break
        elif 'timestamp' in data[group_name]:
            timestamps = data[group_name]['timestamp']
            break

    if timestamps is None:
        print("错误: 找不到时间戳数据")
        return

    # 转换为相对时间（秒）
    relative_time = timestamps - timestamps[0]

    # 如果指定了时间范围，进行裁剪
    if time_range:
        start_sec, end_sec = time_range
        mask = (relative_time >= start_sec) & (relative_time <= end_sec)
        relative_time = relative_time[mask]
    else:
        mask = None

    # 收集所有包含关节数据的组
    joint_groups = {}
    for group_name in data.keys():
        group_data = data[group_name]

        # 查找关节位置数据
        joint_key = None
        for key in ['joint_positions', 'positions', 'joint_position']:
            if key in group_data:
                joint_key = key
                break

        if joint_key:
            joint_data = group_data[joint_key]
            if mask is not None:
                joint_data = joint_data[mask]
            joint_groups[group_name] = {
                'positions': joint_data,
                'velocities': group_data.get('joint_velocities', None),
                'efforts': group_data.get('joint_efforts', None),
            }

    if not joint_groups:
        print("错误: 找不到关节数据")
        return

    # 创建图形
    num_groups = len(joint_groups)
    fig = plt.figure(figsize=(16, 4 * num_groups))
    gs = GridSpec(num_groups, 3, figure=fig, hspace=0.3, wspace=0.3)

    print(f"\n【绘制数据】")
    print(f"时间范围: {relative_time[0]:.2f}s ~ {relative_time[-1]:.2f}s")
    print(f"数据点数: {len(relative_time)}")
    print(f"组件数量: {num_groups}\n")

    # 为每个组件绘制子图
    for idx, (group_name, group_data) in enumerate(joint_groups.items()):
        positions = group_data['positions']
        velocities = group_data['velocities']
        efforts = group_data['efforts']

        num_joints = positions.shape[1] if len(positions.shape) > 1 else 1
        print(f"组件: {group_name}")
        print(f"  关节数: {num_joints}")
        print(f"  数据形状: {positions.shape}")

        # 子图 1: 关节位置
        ax1 = fig.add_subplot(gs[idx, 0])
        if len(positions.shape) > 1:
            for joint_idx in range(num_joints):
                ax1.plot(relative_time, positions[:, joint_idx],
                        label=f'Joint {joint_idx}', alpha=0.7, linewidth=1.5)
        else:
            ax1.plot(relative_time, positions, label='Position', linewidth=1.5)

        ax1.set_xlabel('Time (s)', fontsize=10)
        ax1.set_ylabel('Position (rad)', fontsize=10)
        ax1.set_title(f'{group_name} - Joint Positions', fontsize=12, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=8, ncol=2)
        ax1.grid(True, alpha=0.3)

        # 子图 2: 关节速度
        ax2 = fig.add_subplot(gs[idx, 1])
        if velocities is not None:
            if mask is not None:
                velocities = velocities[mask]
            if len(velocities.shape) > 1:
                for joint_idx in range(num_joints):
                    ax2.plot(relative_time, velocities[:, joint_idx],
                            label=f'Joint {joint_idx}', alpha=0.7, linewidth=1.5)
            else:
                ax2.plot(relative_time, velocities, label='Velocity', linewidth=1.5)
            ax2.set_ylabel('Velocity (rad/s)', fontsize=10)
        else:
            ax2.text(0.5, 0.5, 'No Velocity Data',
                    ha='center', va='center', transform=ax2.transAxes, fontsize=12)

        ax2.set_xlabel('Time (s)', fontsize=10)
        ax2.set_title(f'{group_name} - Joint Velocities', fontsize=12, fontweight='bold')
        if velocities is not None:
            ax2.legend(loc='upper right', fontsize=8, ncol=2)
        ax2.grid(True, alpha=0.3)

        # 子图 3: 关节力矩
        ax3 = fig.add_subplot(gs[idx, 2])
        if efforts is not None:
            if mask is not None:
                efforts = efforts[mask]
            if len(efforts.shape) > 1:
                for joint_idx in range(num_joints):
                    ax3.plot(relative_time, efforts[:, joint_idx],
                            label=f'Joint {joint_idx}', alpha=0.7, linewidth=1.5)
            else:
                ax3.plot(relative_time, efforts, label='Effort', linewidth=1.5)
            ax3.set_ylabel('Effort (Nm)', fontsize=10)
        else:
            ax3.text(0.5, 0.5, 'No Effort Data',
                    ha='center', va='center', transform=ax3.transAxes, fontsize=12)

        ax3.set_xlabel('Time (s)', fontsize=10)
        ax3.set_title(f'{group_name} - Joint Efforts', fontsize=12, fontweight='bold')
        if efforts is not None:
            ax3.legend(loc='upper right', fontsize=8, ncol=2)
        ax3.grid(True, alpha=0.3)

    # 添加总标题
    fig.suptitle('Aligned Robot Joint Data Visualization',
                fontsize=16, fontweight='bold', y=0.995)

    # 保存图片
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    print(f"\n图表已保存到: {output_path}")

    return fig


def plot_single_joint_comparison(data, joint_indices=None,
                                 output_path='joint_comparison.png', dpi=150):
    """
    比较不同组件的特定关节

    Args:
        data: 从 h5 文件加载的数据字典
        joint_indices: 字典，指定每个组件要显示的关节索引
                      例如: {'allegro': [0, 1, 2], 'franka': [0]}
    """

    # 提取时间戳
    timestamps = None
    for group_name in data.keys():
        if 'timestamps' in data[group_name]:
            timestamps = data[group_name]['timestamps']
            break

    if timestamps is None:
        return

    relative_time = timestamps - timestamps[0]

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))

    for group_name in data.keys():
        if 'joint_positions' in data[group_name]:
            positions = data[group_name]['joint_positions']

            if joint_indices and group_name in joint_indices:
                indices = joint_indices[group_name]
            else:
                indices = [0]  # 默认显示第一个关节

            for joint_idx in indices:
                if joint_idx < positions.shape[1]:
                    axes[0].plot(relative_time, positions[:, joint_idx],
                               label=f'{group_name} - Joint {joint_idx}',
                               alpha=0.7, linewidth=2)

        if 'joint_velocities' in data[group_name]:
            velocities = data[group_name]['joint_velocities']

            if joint_indices and group_name in joint_indices:
                indices = joint_indices[group_name]
            else:
                indices = [0]

            for joint_idx in indices:
                if joint_idx < velocities.shape[1]:
                    axes[1].plot(relative_time, velocities[:, joint_idx],
                               label=f'{group_name} - Joint {joint_idx}',
                               alpha=0.7, linewidth=2)

    axes[0].set_xlabel('Time (s)', fontsize=12)
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].set_title('Joint Position Comparison', fontsize=14, fontweight='bold')
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[1].set_title('Joint Velocity Comparison', fontsize=14, fontweight='bold')
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    print(f"关节对比图已保存到: {output_path}")


def main():
    args = parse_args()

    # 确定文件路径
    if args.file:
        h5_path = args.file
    elif args.demo:
        h5_path = os.path.join(args.storage_path, f'demonstration_{args.demo}', 'aligned_data.h5')
    else:
        print("错误: 请提供 --demo 或 --file 参数")
        return

    if not os.path.exists(h5_path):
        print(f"错误: 文件不存在: {h5_path}")
        return

    # 探索文件结构
    structure = inspect_h5_structure(h5_path)

    # 加载数据
    print("\n【加载数据】")
    data = load_aligned_data(h5_path)
    print(f"成功加载 {len(data)} 个数据组")

    # 绘制主可视化
    print("\n【生成可视化】")
    plot_joint_trajectories(data,
                           output_path=args.output,
                           dpi=args.dpi,
                           time_range=args.time_range)

    # 可选: 生成关节对比图
    # 示例: 比较不同机器人的第一个关节
    # plot_single_joint_comparison(data,
    #                             joint_indices={'allegro': [0, 1], 'franka': [0]},
    #                             output_path='joint_comparison.png')

    print("\n完成!")


if __name__ == '__main__':
    main()
