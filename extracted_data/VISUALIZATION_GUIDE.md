# 视频可视化和模型改进指南

## 当前结果

### ✅ 已完成
1. **数据集合并**: demo11 + demo13 作为训练集，demo12 作为验证集
2. **模型训练**: 基础LSTM模型已训练完成
3. **视频可视化**: 已生成带预测标签的视频 `demo12_with_predictions.mp4`

### 📊 当前模型性能

**验证集准确率**: 72.43%

**各类别准确率**:
```
✓ No Action:             94.12% (800 个样本)
✓ Loosely Coupled:       98.26% (2642 个样本)
✗ Unimanual Left:         0.00% (604 个样本)
✗ Unimanual Right:        0.00% (234 个样本)
✗ Tightly Asym (R-Dom):   0.00% (125 个样本)
✗ Tightly Symmetric:      0.00% (219 个样本)
```

### ⚠️ 问题分析

**类别不平衡问题**:
- 训练集中 "Loosely Coupled" 占 51.32%，验证集中占 57.15%
- 模型倾向于预测主导类别，忽略少数类别
- 少数类别（Unimanual、Tightly等）完全被误分类为 Loosely Coupled

## 可视化视频说明

### 视频内容

生成的视频 `demo12_with_predictions.mp4` 包含以下信息：

1. **Frame: X** - 当前帧号
2. **Pred: [协调模式名称]** - 神经网络预测的标签（带颜色）
3. **GT: [协调模式名称]** - 真实标签（Ground Truth）
4. **✓ 或 ✗** - 预测是否正确的标记

### 颜色编码

```
灰色   - No Action
绿色   - Loosely Coupled
蓝色   - Unimanual Left
红色   - Unimanual Right
青色   - Tightly Asym (L-Dom)
洋红   - Tightly Asym (R-Dom)
黄色   - Tightly Symmetric
```

### 前50帧说明

- 前50帧显示 "Initializing (需要50帧历史)"
- 这是因为LSTM需要50帧的时间窗口才能开始预测
- 从第51帧开始显示实际预测结果

## 改进模型性能

### 方法1: 使用类别权重（推荐）

运行改进版训练脚本：

```bash
cd /home/kindred/Desktop/repo/Open-Teach/extracted_data
conda activate openteach
python train_lstm_improved.py
```

**改进点**:
- ✓ 使用加权损失函数 (Weighted Cross-Entropy)
- ✓ 学习率调度器 (ReduceLROnPlateau)
- ✓ 早停机制 (Early Stopping)
- ✓ 更详细的类别准确率监控

**预期效果**:
- 提高少数类别的识别率
- 减少对主导类别的过度依赖

### 方法2: 数据增强

可以考虑：
1. **时间窗口滑动**: 使用更小的步长生成更多训练样本
2. **噪声注入**: 在特征上添加小幅噪声
3. **SMOTE**: 合成少数类别样本

### 方法3: 模型架构改进

```python
# 增加模型复杂度
CONFIG = {
    'hidden_dim': 256,      # 增加隐藏层维度
    'num_layers': 2,        # 使用多层LSTM
    'dropout': 0.3          # 添加Dropout防止过拟合
}
```

## 使用可视化脚本

### 对其他demo生成可视化视频

修改 `visualize_predictions.py` 中的配置：

```python
CONFIG = {
    'demo_path': 'demonstration_13',  # 改为你想可视化的demo
    'video_file': 'cam_0_rgb_video.avi',
    'features_file': 'lstm_danny_features.h5',
    'model_path': 'best_lstm_improved_model.pth',  # 使用改进后的模型
    'scaler_path': 'scaler_improved.pkl',
    'output_video': 'demo13_with_predictions.mp4',
    ...
}
```

然后运行：

```bash
python visualize_predictions.py
```

### 自定义显示样式

在 `visualize_predictions.py` 中可以修改：

1. **字体大小**: 修改 `cv2.FONT_HERSHEY_SIMPLEX` 后的数字
2. **颜色**: 修改 `LABEL_COLORS` 字典
3. **位置**: 修改 `cv2.putText()` 的坐标参数
4. **显示内容**: 在 `create_annotated_video()` 函数中添加/删除文本

## 完整工作流程

### 1. 数据准备（已完成）
```bash
# 每个demo单独处理
cd demonstration_XX
python align_data.py
python merge_labels.py
python feature_generate.py
```

### 2. 合并数据集（已完成）
```bash
cd extracted_data
python merge_datasets.py
```

### 3. 训练基础模型（已完成）
```bash
python train_lstm_merged.py
```

### 4. 生成可视化视频（已完成）
```bash
python visualize_predictions.py
```

### 5. 训练改进模型（推荐下一步）
```bash
python train_lstm_improved.py
```

### 6. 用改进模型重新生成可视化
```bash
# 修改visualize_predictions.py中的模型路径
# model_path': 'best_lstm_improved_model.pth'
# scaler_path': 'scaler_improved.pkl'
python visualize_predictions.py
```

## 评估指标

### 总体准确率 vs 类别准确率

- **总体准确率** 可能会误导，因为主导类别占比大
- **类别准确率** 更能反映模型对每个类别的识别能力
- **F1分数** 可以平衡精确率和召回率

### 混淆矩阵

查看 `visualize_predictions.py` 输出的"预测错误统计"，了解：
- 哪些类别容易混淆
- 模型的主要错误模式

## 常见问题

### Q: 为什么前50帧没有预测？
A: LSTM使用50帧的时间窗口。预测第N帧需要第(N-49)到第N帧的数据。因此前49帧无法构成完整窗口。

### Q: 如何提高少数类别的准确率？
A:
1. 使用类别权重（运行 `train_lstm_improved.py`）
2. 收集更多少数类别的数据
3. 尝试SMOTE或其他过采样技术
4. 调整类别定义，合并相似类别

### Q: 视频生成太慢怎么办？
A:
1. 降低输出视频分辨率
2. 使用GPU加速（如果可用）
3. 减少要处理的帧数（测试时）

### Q: 如何在实时系统中使用？
A:
```python
# 伪代码
buffer = []  # 50帧缓冲区
while True:
    frame_data = get_current_frame_features()
    buffer.append(frame_data)

    if len(buffer) >= 50:
        prediction = model.predict(buffer[-50:])
        display_label(prediction)
        buffer.pop(0)  # 移除最旧的帧
```

## 下一步建议

1. ✅ **运行改进版训练** - `python train_lstm_improved.py`
2. ✅ **重新生成可视化** - 使用改进后的模型
3. **分析混淆矩阵** - 理解模型的错误模式
4. **收集更多数据** - 特别是少数类别
5. **尝试不同架构** - BiLSTM、GRU、Transformer等

## 文件清单

```
extracted_data/
├── merged_dataset.h5                    # 合并后的数据集
├── best_lstm_merged_model.pth           # 基础模型
├── scaler_merged.pkl                    # 基础模型的scaler
├── demo12_with_predictions.mp4          # 可视化视频
├── train_lstm_merged.py                 # 基础训练脚本
├── train_lstm_improved.py               # 改进训练脚本
├── visualize_predictions.py             # 可视化脚本
├── merge_datasets.py                    # 数据合并脚本
└── demonstration_XX/
    ├── aligned_data.h5                  # 对齐的原始数据
    └── lstm_danny_features.h5           # 生成的特征
```
