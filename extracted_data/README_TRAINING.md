# 双手协调分类 - 数据处理和训练流程

## 概述

本项目使用LSTM神经网络对双手协调动作进行分类。数据来自3个演示（demonstration_11, 12, 13）。

## 数据集划分

- **训练集**: demonstration_11 + demonstration_13 (14,585个样本)
- **验证集**: demonstration_12 (4,623个样本)

## 类别定义

```
0: No Action (无动作)
1: Loosely Coupled (松散/弱相关)
2: Unimanual Left (左单手)
3: Unimanual Right (右单手)
4: Tightly Asym (L-Dom) (紧密-左主导)
5: Tightly Asym (R-Dom) (紧密-右主导)
6: Tightly Symmetric (紧密对称)
```

## 特征维度 (8维)

```
1. Hand Offset - 双手距离
2. Hand Offset Velocity - 双手距离变化率
3. Right Trans Velocity - 右手平移速度
4. Right Rot Velocity - 右手旋转速度
5. Left Trans Velocity - 左手平移速度
6. Left Rot Velocity - 左手旋转速度
7. Right Gripper Delta - 右手夹爪变化
8. Left Gripper Delta - 左手夹爪变化
```

## 完整流程

### 1. 数据对齐和标注（每个demo需要单独处理）

```bash
cd demonstration_XX

# 步骤1: 对齐不同传感器的数据到相机时间轴
python align_data.py

# 步骤2: 将Label Studio的标注合并到aligned_data.h5
python merge_labels.py

# 步骤3: 从aligned_data.h5生成LSTM特征
python feature_generate.py
```

**输出文件**:
- `aligned_data.h5` - 包含所有对齐的传感器数据和标签
- `lstm_danny_features.h5` - 包含8维特征和标签

### 2. 合并多个demo的数据集

```bash
cd extracted_data

# 合并demo11和demo13为训练集，demo12为验证集
python merge_datasets.py
```

**输出文件**:
- `merged_dataset.h5` - 包含 X_train, Y_train, X_val, Y_val

**重要**: 此脚本会自动裁剪每个demo的前50帧，因为LSTM使用50帧的时间窗口。

### 3. 训练LSTM模型

```bash
cd extracted_data

# 训练模型
python train_lstm_merged.py
```

**输出文件**:
- `best_lstm_merged_model.pth` - 最佳模型权重
- `scaler_merged.pkl` - 数据标准化的scaler（推理时需要）
- `training_curves_merged.png` - 训练曲线图

## 训练配置

```python
CONFIG = {
    'window_size': 50,      # LSTM时间窗口
    'input_dim': 8,         # 特征维度
    'hidden_dim': 128,      # LSTM隐藏层维度
    'num_classes': 7,       # 类别数量 (0-6)
    'batch_size': 32,
    'learning_rate': 0.001,
    'num_epochs': 50
}
```

## 数据统计

### 训练集标签分布

```
类别 0: 2,352 个样本 (16.13%)
类别 1: 7,485 个样本 (51.32%)  <- 主导类别
类别 2: 2,549 个样本 (17.48%)
类别 3: 1,030 个样本 (7.06%)
类别 4: 269 个样本 (1.84%)
类别 5: 251 个样本 (1.72%)
类别 6: 649 个样本 (4.45%)
```

### 验证集标签分布

```
类别 0: 799 个样本 (17.28%)
类别 1: 2,642 个样本 (57.15%)  <- 主导类别
类别 2: 604 个样本 (13.07%)
类别 3: 234 个样本 (5.06%)
类别 5: 125 个样本 (2.70%)
类别 6: 219 个样本 (4.74%)

注意: 验证集中没有类别4的样本
```

## 注意事项

1. **数据不平衡**: 类别1（Loosely Coupled）占50%以上，可能需要考虑类别权重
2. **缺失类别**: 验证集中没有类别4，模型对该类别的泛化能力无法评估
3. **窗口大小**: 前50帧被裁剪掉，因为LSTM需要50帧的历史数据
4. **数据标准化**: 使用StandardScaler对特征进行标准化，推理时需要使用相同的scaler

## 添加新的演示数据

如果要添加新的demonstration_XX:

```bash
# 1. 确保目录包含以下原始数据文件:
#    - cam_0_rgb_video.metadata
#    - left_franka_cartesian_states.h5
#    - right_franka_cartesian_states.h5
#    - left tesollo_joint_states.h5
#    - right tesollo_joint_states.h5
#    等等...

# 2. 从demonstration_11复制处理脚本
cp demonstration_11/{align_data.py,merge_labels.py,feature_generate.py} demonstration_XX/

# 3. 修改merge_labels.py中的配置:
#    - JSON_PATH: 你的Label Studio导出文件
#    - TOTAL_FRAMES: 视频总帧数

# 4. 运行处理流程
cd demonstration_XX
python align_data.py
python merge_labels.py
python feature_generate.py

# 5. 修改merge_datasets.py，添加新demo到训练或验证集
# 6. 重新合并数据集并训练
```

## 推理使用

训练完成后，使用模型进行推理时需要:

1. 加载模型: `model.load_state_dict(torch.load('best_lstm_merged_model.pth'))`
2. 加载scaler: `scaler = joblib.load('scaler_merged.pkl')`
3. 标准化输入: `X_scaled = scaler.transform(X)`
4. 创建50帧窗口并预测

## 问题排查

### Q: demonstration_12之前为什么X形状不匹配？
A: 标签设置为4675帧，但实际数据只有4674帧。已修复TOTAL_FRAMES为4674。

### Q: 为什么要裁剪前50帧？
A: LSTM使用50帧的时间窗口。例如，要预测第50帧的标签，需要第0-49帧作为输入。因此前49帧无法构成完整窗口。为了简化，我们从第50帧开始。

### Q: 如何调整验证集？
A: 修改 `merge_datasets.py` 中的 `DEMO_PATHS` 字典，将demo分配到 'train' 或 'val' 列表。
