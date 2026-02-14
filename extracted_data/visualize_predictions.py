"""
将LSTM模型预测结果可视化到视频上
- 在每一帧上显示预测的协调模式名称
- 从第51帧开始（前50帧用于LSTM窗口）
"""
import torch
import torch.nn as nn
import h5py
import numpy as np
import cv2
import joblib
from pathlib import Path

# === 1. 修改配置字典 ===
CONFIG = {
    'demo_path': 'demonstration_63',           # 目标文件夹
    'video_file': 'cam_0_rgb_video.avi',
    'features_file': 'lstm_danny_features.h5', # 确保这是包含2维特征的文件
    'model_path': '213grasp_model.pth',    # 你刚刚训练好的二元模型
    'scaler_path': '213scaler_merged.pkl',        # 对应的标准化器
    'output_video': 'demo63_213results.mp4',
    'window_size': 50,
    'input_dim': 25,                            # 关键：改为2维 (L/R error)
    'hidden_dim': 128,                          # 关键：改为你二元训练时的64
    'layer_dim': 1,
    'num_classes': 7,                          # 
    'device': 'cuda' if torch.cuda.is_available() else 'cpu'
}


LABEL_NAMES = {
    0: "No Action",
    1: "Loosely Coupled",
    2: "Unimanual Left",
    3: "Unimanual Right",
    4: "Tightly Asym (L-Dom)",
    5: "Tightly Asym (R-Dom)",
    6: "Tightly Symmetric"
}

LABEL_COLORS = {
    0: (128, 128, 128),  # 灰色
    1: (0, 255, 0),      # 绿色
    2: (255, 0, 0),      # 蓝色
    3: (0, 0, 255),      # 红色
    4: (255, 255, 0),    # 青色
    5: (255, 0, 255),    # 洋红
    6: (0, 255, 255)
}

# === LSTM模型定义 ===
class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes, num_layers):
        super(SimpleLSTM, self).__init__()
        # 如果你训练时用了 num_layers=1，这里也要保持一致
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True, num_layers=num_layers)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        out, (h_n, c_n) = self.lstm(x)
        # 取最后一个时间步的输出
        last_step_out = out[:, -1, :] 
        logits = self.fc(last_step_out)
        return logits


def load_model_and_scaler():
    """加载训练好的模型和scaler"""
    print("正在加载模型和scaler...")

    # 加载scaler
    scaler = joblib.load(CONFIG['scaler_path'])

    # 创建模型
    model = SimpleLSTM(
        input_dim=CONFIG['input_dim'],
        hidden_dim=CONFIG['hidden_dim'],
        num_classes=CONFIG['num_classes'],
        num_layers=CONFIG['layer_dim']  
    ).to(CONFIG['device'])

    # 加载权重
    model.load_state_dict(torch.load(CONFIG['model_path'], map_location=CONFIG['device']))
    model.eval()

    print(f"模型已加载到: {CONFIG['device']}")
    return model, scaler

def load_features():
    """加载特征数据"""
    feature_path = Path(CONFIG['demo_path']) / CONFIG['features_file']
    print(f"正在加载特征: {feature_path}")

    with h5py.File(feature_path, 'r') as f:
        X = f['X'][:]
        Y = f['Y'][:]

    print(f"特征形状: X={X.shape}, Y={Y.shape}")
    return X, Y

def predict_all_frames(model, scaler, X):
    """
    对所有帧进行预测

    注意：前50帧没有足够的历史数据，无法预测
    从第50帧（索引49）开始，每一帧都使用前50帧作为输入
    """
    window_size = CONFIG['window_size']
    predictions = []

    print("正在进行推理...")

    # 标准化特征
    X_scaled = scaler.transform(X)
    X_tensor = torch.FloatTensor(X_scaled).to(CONFIG['device'])

    with torch.no_grad():
        # 从第window_size帧开始预测
        for i in range(window_size - 1, len(X)):
            # 获取窗口: [i-window_size+1, i+1)
            window = X_tensor[i - window_size + 1 : i + 1].unsqueeze(0)

            # 预测
            output = model(window)
            pred = torch.argmax(output, dim=1).item()
            predictions.append(pred)

    print(f"完成推理，共预测 {len(predictions)} 帧")
    return predictions

def create_annotated_video(video_path, predictions, ground_truth):
    """
    创建带标注的视频

    Args:
        video_path: 原始视频路径
        predictions: 预测结果列表（从第50帧开始）
        ground_truth: 真实标签列表（从第1帧开始，已裁剪差分）
    """
    print(f"正在读取视频: {video_path}")
    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise ValueError(f"无法打开视频: {video_path}")

    # 获取视频属性
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"视频信息: {width}x{height} @ {fps}fps, 总帧数: {total_frames}")

    # 创建输出视频
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(CONFIG['output_video'], fourcc, fps, (width, height))

    print(f"正在生成标注视频: {CONFIG['output_video']}")

    frame_idx = 0
    processed_frames = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 前50帧：显示"Initializing..."
        if frame_idx < CONFIG['window_size']:
            text = "Initializing (需要50帧历史)"
            color = (200, 200, 200)
            gt_text = ""
        else:
            # 从第50帧开始，predictions的索引是frame_idx - window_size
            pred_idx = frame_idx - CONFIG['window_size']

            if pred_idx < len(predictions):
                pred_label = predictions[pred_idx]
                pred_name = LABEL_NAMES[pred_label]
                color = LABEL_COLORS[pred_label]
                text = f"Pred: {pred_name}"

                # 显示真实标签（用于对比）
                if pred_idx < len(ground_truth):
                    gt_label = ground_truth[pred_idx]
                    gt_name = LABEL_NAMES[gt_label]
                    gt_text = f"GT: {gt_name}"

                    # 如果预测正确，标记为绿色，否则红色
                    if pred_label == gt_label:
                        gt_color = (0, 255, 0)  # 绿色
                        match_text = "✓"
                    else:
                        gt_color = (0, 0, 255)  # 红色
                        match_text = "✗"
                else:
                    gt_text = ""
                    gt_color = (255, 255, 255)
                    match_text = ""
            else:
                text = ""
                color = (255, 255, 255)
                gt_text = ""

        # 在帧上添加文本
        # 1. 帧号
        cv2.putText(frame, f"Frame: {frame_idx + 1}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 2. 预测结果（大字体，中间位置）
        if frame_idx >= CONFIG['window_size'] and pred_idx < len(predictions):
            # 预测标签
            cv2.putText(frame, text,
                        (10, height - 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)

            # 真实标签
            if gt_text:
                cv2.putText(frame, gt_text,
                            (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, gt_color, 2)

                # 匹配标记
                cv2.putText(frame, match_text,
                            (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, gt_color, 3)
        else:
            cv2.putText(frame, text,
                        (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        out.write(frame)
        frame_idx += 1
        processed_frames += 1

        if processed_frames % 100 == 0:
            print(f"  已处理 {processed_frames}/{total_frames} 帧")

    cap.release()
    out.release()

    print(f"视频生成完成: {CONFIG['output_video']}")
    print(f"总共处理了 {processed_frames} 帧")

def calculate_accuracy(predictions, ground_truth):
    """计算准确率"""
    # predictions和ground_truth长度应该相同
    min_len = min(len(predictions), len(ground_truth))
    predictions = np.array(predictions[:min_len])
    ground_truth = np.array(ground_truth[:min_len])

    accuracy = np.mean(predictions == ground_truth) * 100

    # 计算每个类别的准确率
    print("\n" + "=" * 60)
    print("预测结果统计:")
    print("=" * 60)
    print(f"总体准确率: {accuracy:.2f}%")
    print(f"总样本数: {min_len}")

    print("\n各类别准确率:")
    for label_id, label_name in LABEL_NAMES.items():
        mask = ground_truth == label_id
        if mask.sum() > 0:
            class_acc = np.mean(predictions[mask] == ground_truth[mask]) * 100
            print(f"  {label_name}: {class_acc:.2f}% ({mask.sum()} 个样本)")

    # 混淆矩阵简要统计
    print("\n预测错误统计 (Top 5):")
    errors = {}
    for i in range(min_len):
        if predictions[i] != ground_truth[i]:
            key = f"{LABEL_NAMES[ground_truth[i]]} -> {LABEL_NAMES[predictions[i]]}"
            errors[key] = errors.get(key, 0) + 1

    for error_type, count in sorted(errors.items(), key=lambda x: x[1], reverse=True)[:5]:
        print(f"  {error_type}: {count} 次")

def main():
    print("=" * 60)
    print("视频预测可视化工具")
    print("=" * 60)

    # 1. 加载模型
    model, scaler = load_model_and_scaler()

    X, Y = load_features()



    predictions = predict_all_frames(model, scaler, X)
    
    # 这里的索引对齐：
    # 因为 X 和 Y 在 feature_generate 中都做了 [1:] 切片，
    # 所以 Y 的第 49 个元素对应视频的第 50 帧
    ground_truth = Y[CONFIG['window_size'] - 1:] 
    
    calculate_accuracy(predictions, ground_truth)
    
    video_path = Path(CONFIG['demo_path']) / CONFIG['video_file']
    create_annotated_video(video_path, predictions, ground_truth)

if __name__ == "__main__":
    main()
