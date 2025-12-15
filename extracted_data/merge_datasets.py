"""
合并多个演示的特征数据集
- demo11 和 demo13 作为训练集
- demo12 作为验证集
- 裁剪前50帧（因为LSTM窗口大小为50）
"""
import h5py
import numpy as np
import os

# === 配置 ===
DEMO_PATHS = {
    'train': [
        'demonstration_11/lstm_danny_features.h5',
        'demonstration_13/lstm_danny_features.h5'
    ],
    'val': [
        'demonstration_12/lstm_danny_features.h5'
    ]
}

WINDOW_SIZE = 50  # LSTM窗口大小
OUTPUT_FILE = 'merged_dataset.h5'

def load_and_trim_data(file_path, window_size):
    """
    加载特征数据并裁剪前window_size帧

    Args:
        file_path: h5文件路径
        window_size: 要裁剪的帧数

    Returns:
        X_trimmed, Y_trimmed: 裁剪后的特征和标签
    """
    print(f"  正在加载: {file_path}")

    with h5py.File(file_path, 'r') as f:
        X = f['X'][:]
        Y = f['Y'][:]

        print(f"    原始形状: X={X.shape}, Y={Y.shape}")

        # 裁剪前window_size帧
        X_trimmed = X[window_size:]
        Y_trimmed = Y[window_size:]

        print(f"    裁剪后形状: X={X_trimmed.shape}, Y={Y_trimmed.shape}")

        return X_trimmed, Y_trimmed

def merge_datasets(demo_paths, window_size):
    """
    合并多个演示的数据集

    Args:
        demo_paths: 演示文件路径列表
        window_size: LSTM窗口大小

    Returns:
        X_merged, Y_merged: 合并后的特征和标签
    """
    X_list = []
    Y_list = []

    for path in demo_paths:
        X, Y = load_and_trim_data(path, window_size)
        X_list.append(X)
        Y_list.append(Y)

    # 沿着第0维（样本维）拼接
    X_merged = np.concatenate(X_list, axis=0)
    Y_merged = np.concatenate(Y_list, axis=0)

    return X_merged, Y_merged

def main():
    print("=" * 60)
    print("开始合并数据集...")
    print("=" * 60)

    # 1. 合并训练集 (demo11 + demo13)
    print("\n[1/2] 合并训练集 (demo11 + demo13)...")
    X_train, Y_train = merge_datasets(DEMO_PATHS['train'], WINDOW_SIZE)
    print(f"\n训练集总计: X={X_train.shape}, Y={Y_train.shape}")

    # 2. 处理验证集 (demo12)
    print("\n[2/2] 处理验证集 (demo12)...")
    X_val, Y_val = merge_datasets(DEMO_PATHS['val'], WINDOW_SIZE)
    print(f"\n验证集总计: X={X_val.shape}, Y={Y_val.shape}")

    # 3. 保存合并后的数据集
    print(f"\n正在保存至: {OUTPUT_FILE}...")
    with h5py.File(OUTPUT_FILE, 'w') as f:
        # 训练集
        f.create_dataset('X_train', data=X_train, compression="gzip")
        f.create_dataset('Y_train', data=Y_train, dtype='i8', compression="gzip")

        # 验证集
        f.create_dataset('X_val', data=X_val, compression="gzip")
        f.create_dataset('Y_val', data=Y_val, dtype='i8', compression="gzip")

        # 元数据
        f.attrs['window_size'] = WINDOW_SIZE
        f.attrs['train_demos'] = 'demo11, demo13'
        f.attrs['val_demos'] = 'demo12'
        f.attrs['feature_columns'] = "1:Offset, 2:OffsetVel, 3:R_Trans, 4:R_Rot, 5:L_Trans, 6:L_Rot, 7:R_Grip, 8:L_Grip"

        print(f"成功保存！")

    # 4. 验证
    print("\n" + "=" * 60)
    print("验证合并结果:")
    print("=" * 60)
    with h5py.File(OUTPUT_FILE, 'r') as f:
        print(f"数据集包含的key: {list(f.keys())}")
        print(f"\nX_train shape: {f['X_train'].shape}")
        print(f"Y_train shape: {f['Y_train'].shape}")
        print(f"X_val shape: {f['X_val'].shape}")
        print(f"Y_val shape: {f['Y_val'].shape}")

        print(f"\n元数据:")
        for key, value in f.attrs.items():
            print(f"  {key}: {value}")

        # 统计标签分布
        print(f"\n训练集标签分布:")
        unique, counts = np.unique(f['Y_train'][:], return_counts=True)
        for label, count in zip(unique, counts):
            print(f"  类别 {label}: {count} 个样本 ({count/len(f['Y_train'][:])*100:.2f}%)")

        print(f"\n验证集标签分布:")
        unique, counts = np.unique(f['Y_val'][:], return_counts=True)
        for label, count in zip(unique, counts):
            print(f"  类别 {label}: {count} 个样本 ({count/len(f['Y_val'][:])*100:.2f}%)")

    print("\n" + "=" * 60)
    print("数据集合并完成！")
    print("=" * 60)

if __name__ == "__main__":
    main()
