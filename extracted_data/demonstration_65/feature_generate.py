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

def calc_rot_6d_velocity(rot_seq):
    """
    计算旋转的 6D 变化率
    Input: (N, 3, 3) 旋转矩阵序列
    Output: (N-1, 6) 6D 旋转向量的变化率
    """
    # 1. 提取 6D 表示：取旋转矩阵的前两列
    # rot_seq[:, :, 0] 是第一列 (N, 3)
    # rot_seq[:, :, 1] 是第二列 (N, 3)
    # 使用 reshape 或 concatenate 合并为 (N, 6)
    rot_6d = rot_seq[:, :, :2].transpose(0, 2, 1).reshape(-1, 6)
    
    # 2. 计算差分 (变化率)
    # delta_6d = R6d[t] - R6d[t-1]
    rot_6d_diff = rot_6d[1:] - rot_6d[:-1]
    
    return rot_6d_diff # 返回 (N-1, 6)

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

def calc_relative_features_rate(pos_L, pos_R, rot_L, rot_R):
    """
    计算相对位移和相对旋转的变化率
    pos: (N, 3)
    rot: (N, 3, 3) 旋转矩阵
    Output: 相对位移变化率 (N-1, 3), 相对旋转(6D)变化率 (N-1, 6)
    """
    N = pos_L.shape[0]
    
    # --- 1. 计算每一时刻的相对局部位置 (N, 3) ---
    # pos_diff: 全局下的位移差 (左手指向右手)
    pos_diff = pos_R - pos_L 
    # 将 pos_diff 投影到左手的局部坐标系中
    # 结果代表：以左手为原点时，右手的坐标
    pos_local = np.einsum('nij,ni->nj', rot_L.transpose(0, 2, 1), pos_diff)
    
    # 计算位移变化率 (N-1, 3)
    pos_rel_rate = pos_local[1:] - pos_local[:-1]

    # --- 2. 计算每一时刻的相对旋转矩阵 (N, 3, 3) ---
    # R_rel = R_L^T * R_R
    rot_rel = np.einsum('nij,njk->nik', rot_L.transpose(0, 2, 1), rot_R)
    
    # 提取 6D 表示 (取前两列) -> (N, 6)
    # rot_rel[:, :, 0] 是第一列, rot_rel[:, :, 1] 是第二列
    rot_6d = rot_rel[:, :, :2].reshape(N, 6)
    
    # 计算旋转变化率 (N-1, 6)
    rot_rel_rate = rot_6d[1:] - rot_6d[:-1]
    
    return pos_rel_rate, rot_rel_rate

def main():
    print(f"正在处理 {INPUT_FILE} ...")
    
    with h5py.File(INPUT_FILE, 'r') as f:
        # --- 1. 读取基础数据 ---
        # 假设 Right 是 Dominant, Left 是 Non-Dominant
        # 根据你之前的文件名，Right是主手
        
        # 位置 (N, 3)
        pos_R = f['right_gripper']['wrist_positions'][:8800]
        pos_L = f['left_gripper']['wrist_positions'][:8800]
        
        # 方向 (N, 3, 3)
        ori_R = f['right_gripper']['wrist_orientations'][:8800]
        ori_L = f['left_gripper']['wrist_orientations'][:8800]
        
        # Gripper (N, 12)
        joints_L = f['left_gripper']['actual_positions'][:8800]
        joints_R = f['right_gripper']['commanded_positions'][:8800]

        error_R = f['right_gripper']['error_positions'][:8800]
        error_L = f['left_gripper']['error_positions'][:8800]

        # 标签 (N, )
        labels = f['coordination_labels'][:8800]
        
        N = len(labels)
        print(f"总帧数: {N}")

    # --- 2. 构造特征 (Feature 1-8) ---
    
    # Feature 1: Hand Offset (Distance between hands)
    # Shape: (N, )
    hand_offset_vec = pos_L - pos_R
    hand_offset = np.linalg.norm(hand_offset_vec, axis=1)

    #contact information
    
    mean_error_R = np.mean(np.maximum(error_R, 0), axis=1)
    mean_error_L = np.mean(np.maximum(error_L, 0), axis=1)

    feat_9 = mean_error_R[1:].reshape(-1, 1)
    feat_10 = mean_error_L[1:].reshape(-1, 1)

    feat_11, feat_12 = calc_relative_features_rate(pos_L, pos_R, ori_L, ori_R)
    
    # 截取从第1帧开始（因为后面都要做差分，会少一帧）
    # F1: Hand Offset (对应时刻 t) -> 取 [1:]
    feat_1 = hand_offset[1:].reshape(-1, 1)
    
    # Feature 2: Hand Offset Velocity (Rate of change)
    # Diff of Feature 1
    feat_2 = (hand_offset[1:] - hand_offset[:-1]).reshape(-1, 1)
    
    # Feature 3: Dom (Right) Trans Velocity
    feat_3 = calc_trans_velocity(pos_R)
    
    # Feature 4: Dom (Right) Rot Velocity
    feat_4 = calc_rot_6d_velocity(ori_R)
    
    # Feature 5: Non-Dom (Left) Trans Velocity
    feat_5 = calc_trans_velocity(pos_L)
    
    # Feature 6: Non-Dom (Left) Rot Velocity
    feat_6 = calc_rot_6d_velocity(ori_L)
    
    # Feature 7: Dom (Right) Gripper Change (你定义的)
    # 注意：Danny论文里是 Dom在前，所以这里我把 Right 放前面
    feat_7 = calc_gripper_diff(joints_R)
    
    # Feature 8: Non-Dom (Left) Gripper Change
    feat_8 = calc_gripper_diff(joints_L)
    
    # --- 3. 组合与保存 ---
    
    # 拼接 X
    X = np.hstack([
        feat_9, feat_10, feat_3, feat_4, feat_5, feat_6, feat_11, feat_12          # 相对关系
    ])
    
    # 处理 Y (去掉第0帧)
    Y = labels[1:]
    
    print("\n=== 最终数据形状 ===")
    print(f"X: {X.shape} (期望: [{N-1}, 25])")
    print(f"Y: {Y.shape} (期望: [{N-1}, ])")
    
    with h5py.File(OUTPUT_FILE, 'w') as out:
        out.create_dataset('X', data=X, compression="gzip")
        out.create_dataset('Y', data=Y, dtype='i8', compression="gzip")
        
        # 记录每一列的含义
        col_desc = "1:Right error, 2:Left error, 3:Right trans velo, 4:Right rot velo, 5:Left trans velo, 6:Left rot velo, 7: , 8:"
        out.attrs['columns'] = col_desc
        print(f"文件已保存至: {OUTPUT_FILE}")
        print(f"列定义: {col_desc}")

if __name__ == "__main__":
    main()