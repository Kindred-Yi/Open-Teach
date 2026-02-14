import h5py
import numpy as np

# === 配置 ===
INPUT_FILE = "aligned_data.h5"
OUTPUT_FILE = "lstm_danny_features.h5"
#OUTPUT_FILE = "actu_lstm_danny_features.h5"


def calc_trans_velocity(pos_seq):
    """
    计算平移速度 (位置差的模长)
    Input: (N, 3)
    Output: (N-1, 1)
    """
    # delta = p[t] - p[t-1]
    deltas = pos_seq[1:] - pos_seq[:-1]
    # norm(delta)
    # keepdims=True 保持 (N-1, 1) 形状方便拼接
    return np.linalg.norm(deltas, axis=1, keepdims=True)

def calc_rot_velocity(ori_seq):
    """
    计算旋转速度 (四元数角度差)
    Input: (N, 4)  [x, y, z, w] or [w, x, y, z]
    Output: (N-1, 1)
    """
    q1 = ori_seq[:-1]
    q2 = ori_seq[1:]
    
    # 计算点积 <q1, q2>
    # sum(q1 * q2, axis=1)
    dot_product = np.sum(q1 * q2, axis=1)
    
    # 四元数特性：q 和 -q 表示相同的旋转
    # 所以我们要取 abs(dot_product) 来确保走最短路径
    dot_product = np.abs(dot_product)
    
    # 防止数值误差导致 arccos 越界
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # theta = 2 * arccos(|<q1, q2>|)
    theta = 2 * np.arccos(dot_product)
    
    return theta.reshape(-1, 1)

def calc_gripper_diff(joint_seq):
    """
    计算 Gripper 开合度变化
    Input: (N, 12)
    Output: (N-1, 1)
    """
    # 1. 计算 C_t (所有关节均值)
    c_t = np.mean(joint_seq, axis=1)
    # 2. 差分绝对值
    diff = np.abs(c_t[1:] - c_t[:-1])
    return diff.reshape(-1, 1)

def main():
    print(f"正在处理 {INPUT_FILE} ...")
    
    with h5py.File(INPUT_FILE, 'r') as f:
        # --- 1. 读取基础数据 ---
        # 假设 Right 是 Dominant, Left 是 Non-Dominant
        # 根据你之前的文件名，Right是主手
        
        # 位置 (N, 3)
        pos_R = f['right_arm_state']['cartesian_positions'][:]
        pos_L = f['left_arm_state']['cartesian_positions'][:]
        
        # 方向 (N, 4)
        ori_R = f['right_arm_state']['cartesian_orientations'][:]
        ori_L = f['left_arm_state']['cartesian_orientations'][:]
        
        # Gripper (N, 12)
        joints_R = f['right_gripper']['actual_positions'][:]
        joints_L = f['left_gripper']['actual_positions'][:]
        # joints_R = f['right_gripper']['commanded_positions'][:]
        # joints_L = f['left_gripper']['commanded_positions'][:]
        error_R = f['right_gripper']['error_positions'][:]
        error_L = f['left_gripper']['error_positions'][:]

        # 标签 (N, )
        labels = f['coordination_labels'][:]
        
        N = len(labels)
        print(f"总帧数: {N}")

    # --- 2. 构造特征 (Feature 1-8) ---
    
    # Feature 1: Hand Offset (Distance between hands)
    # Shape: (N, )
    hand_offset_vec = pos_L - pos_R
    hand_offset = np.linalg.norm(hand_offset_vec, axis=1)

    #contact information
    
    mean_error_R = np.mean(error_R, axis=1)
    mean_error_L = np.mean(error_L, axis=1)

    feat_9 = mean_error_R[1:].reshape(-1, 1)
    feat_10 = mean_error_L[1:].reshape(-1, 1)

    
    # 截取从第1帧开始（因为后面都要做差分，会少一帧）
    # F1: Hand Offset (对应时刻 t) -> 取 [1:]
    feat_1 = hand_offset[1:].reshape(-1, 1)
    
    # Feature 2: Hand Offset Velocity (Rate of change)
    # Diff of Feature 1
    feat_2 = (hand_offset[1:] - hand_offset[:-1]).reshape(-1, 1)
    
    # Feature 3: Dom (Right) Trans Velocity
    feat_3 = calc_trans_velocity(pos_R)
    
    # Feature 4: Dom (Right) Rot Velocity
    feat_4 = calc_rot_velocity(ori_R)
    
    # Feature 5: Non-Dom (Left) Trans Velocity
    feat_5 = calc_trans_velocity(pos_L)
    
    # Feature 6: Non-Dom (Left) Rot Velocity
    feat_6 = calc_rot_velocity(ori_L)
    
    # Feature 7: Dom (Right) Gripper Change (你定义的)
    # 注意：Danny论文里是 Dom在前，所以这里我把 Right 放前面
    feat_7 = calc_gripper_diff(joints_R)
    
    # Feature 8: Non-Dom (Left) Gripper Change
    feat_8 = calc_gripper_diff(joints_L)
    
    # --- 3. 组合与保存 ---
    
    # 拼接 X
    X = np.hstack([
        feat_9, feat_10          # 相对关系
    ])
    
    # 处理 Y (去掉第0帧)
    Y = labels[1:]
    
    print("\n=== 最终数据形状 ===")
    print(f"X: {X.shape} (期望: [{N-1}, 2])")
    print(f"Y: {Y.shape} (期望: [{N-1}, ])")
    
    # 特征顺序说明 (保存为 Attribute)
    feature_names = [
        "1. Hand Offset",
        "2. Hand Offset Velocity",
        "3. Dom(R) Trans Vel",
        "4. Dom(R) Rot Vel",
        "5. Dom(R) Gripper Delta",  # 注意我把Gripper放到了手对应的位置附近，或者放最后
        "6. Non-Dom(L) Trans Vel",
        "7. Non-Dom(L) Rot Vel",
        "8. Non-Dom(L) Gripper Delta"
    ]
    # 上面的代码 stack 顺序是: 1, 2, 3, 4, 7(RG), 5, 6, 8(LG)
    # 让我调整一下顺序，完全贴合你的描述：前6个是论文的，后2个是Gripper
    
    X_final = np.hstack([
        feat_1, feat_2,  # (1) Offset, (2) Offset Rate
        feat_3, feat_4,  # (3) Dom Trans, (4) Dom Rot
        feat_5, feat_6,  # (5) Non-Dom Trans, (6) Non-Dom Rot
        feat_7, feat_8   # (7) Dom Grip, (8) Non-Dom Grip
    ])
    
    print(f"X_final: {X.shape}")

    with h5py.File(OUTPUT_FILE, 'w') as out:
        out.create_dataset('X', data=X, compression="gzip")
        out.create_dataset('Y', data=Y, dtype='i8', compression="gzip")
        
        # 记录每一列的含义
        col_desc = "1:Right error, 2:Left error"
        out.attrs['columns'] = col_desc
        print(f"文件已保存至: {OUTPUT_FILE}")
        print(f"列定义: {col_desc}")

if __name__ == "__main__":
    main()