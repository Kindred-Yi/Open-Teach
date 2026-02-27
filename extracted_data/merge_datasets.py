import h5py
import numpy as np
import os
from sklearn.preprocessing import StandardScaler
import joblib

# === 配置 ===
DEMO_PATHS = {
    'train': [
        'demonstration_60/lstm_danny_features.h5', 'demonstration_61/lstm_danny_features.h5', 'demonstration_62/lstm_danny_features.h5', 'demonstration_64/lstm_danny_features.h5', 'demonstration_65/lstm_danny_features.h5', 'demonstration_66/lstm_danny_features.h5'
    ],
    'val': [
        'demonstration_63/lstm_danny_features.h5'
    ]
}

WINDOW_SIZE = 50  # 历史窗口长度
OUTPUT_FILE = '219merged_dataset_3d.h5'
SCALER_FILE = '219scaler_merged.pkl'

def create_windows(X, Y, window_size):
    """
    在单个演示数据内滑动窗口
    """
    X_3d = []
    Y_win = []
    # 从第 window_size - 1 帧开始，每一帧都有足够的历史数据
    for i in range(window_size - 1, len(X)):
        X_3d.append(X[i - window_size + 1 : i + 1])
        Y_win.append(Y[i])
    return np.array(X_3d), np.array(Y_win)

def main():
    print("开始生成 3D 窗口化数据集...")
    
    # --- 1. 拟合标准化器 (Scaler) ---
    # 为了保证训练和验证集尺度一致，我们必须先用原始训练数据 fit scaler
    all_raw_train_X = []
    for path in DEMO_PATHS['train']:
        with h5py.File(path, 'r') as f:
            all_raw_train_X.append(f['X'][:])
    
    X_train_concat = np.concatenate(all_raw_train_X, axis=0)
    scaler = StandardScaler()
    scaler.fit(X_train_concat)
    joblib.dump(scaler, SCALER_FILE)
    print(f"✓ Scaler 已保存至 {SCALER_FILE}")

    # --- 2. 处理训练集 ---
    final_train_X, final_train_Y = [], []
    for path in DEMO_PATHS['train']:
        with h5py.File(path, 'r') as f:
            X_scaled = scaler.transform(f['X'][:])
            Y_raw = f['Y'][:]
            X_win, Y_win = create_windows(X_scaled, Y_raw, WINDOW_SIZE)
            final_train_X.append(X_win)
            final_train_Y.append(Y_win)
    
    X_train_final = np.concatenate(final_train_X, axis=0)
    Y_train_final = np.concatenate(final_train_Y, axis=0)

    # --- 3. 处理验证集 ---
    final_val_X, final_val_Y = [], []
    for path in DEMO_PATHS['val']:
        with h5py.File(path, 'r') as f:
            X_scaled = scaler.transform(f['X'][:])
            Y_raw = f['Y'][:]
            X_win, Y_win = create_windows(X_scaled, Y_raw, WINDOW_SIZE)
            final_val_X.append(X_win)
            final_val_Y.append(Y_win)
            
    X_val_final = np.concatenate(final_val_X, axis=0)
    Y_val_final = np.concatenate(final_val_Y, axis=0)

    # --- 4. 保存 3D 数据 ---
    with h5py.File(OUTPUT_FILE, 'w') as f:
        f.create_dataset('X_train', data=X_train_final, compression="gzip")
        f.create_dataset('Y_train', data=Y_train_final, compression="gzip")
        f.create_dataset('X_val', data=X_val_final, compression="gzip")
        f.create_dataset('Y_val', data=Y_val_final, compression="gzip")
        f.attrs['window_size'] = WINDOW_SIZE

    print(f"完成！训练窗口数: {len(X_train_final)}, 验证窗口数: {len(X_val_final)}")

if __name__ == "__main__":
    main()