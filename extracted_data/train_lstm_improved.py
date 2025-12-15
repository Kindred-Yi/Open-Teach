"""
改进版LSTM训练脚本 - 解决类别不平衡问题
- 使用类别权重平衡损失函数
- 增加学习率调度器
- 添加早停机制
"""
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import h5py
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.utils.class_weight import compute_class_weight
import matplotlib.pyplot as plt
import joblib

# === 配置 ===
CONFIG = {
    'input_file': 'merged_dataset.h5',
    'window_size': 50,
    'input_dim': 8,
    'hidden_dim': 128,
    'num_classes': 7,
    'batch_size': 32,
    'learning_rate': 0.001,
    'num_epochs': 100,
    'patience': 15,  # 早停耐心值
    'device': 'cuda' if torch.cuda.is_available() else 'cpu',
    'save_model_path': 'best_lstm_improved_model.pth',
    'save_scaler_path': 'scaler_improved.pkl'
}

# === 1. 数据集类 ===
class BimanualDataset(Dataset):
    def __init__(self, X, Y, window_size):
        self.X = torch.FloatTensor(X)
        self.Y = torch.LongTensor(Y)
        self.window_size = window_size

    def __len__(self):
        return len(self.X) - self.window_size + 1

    def __getitem__(self, idx):
        x_seq = self.X[idx : idx + self.window_size]
        y_label = self.Y[idx + self.window_size - 1]
        return x_seq, y_label

# === 2. LSTM模型 ===
class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes):
        super(SimpleLSTM, self).__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        out, _ = self.lstm(x)
        last_step_out = out[:, -1, :]
        logits = self.fc(last_step_out)
        return logits

# === 3. 计算类别权重 ===
def compute_class_weights(Y_train):
    """
    计算类别权重来处理不平衡数据
    """
    # 统计每个类别的样本数
    unique_classes = np.unique(Y_train)
    class_weights = compute_class_weight(
        class_weight='balanced',
        classes=unique_classes,
        y=Y_train
    )

    # 创建完整的权重数组（包括所有可能的类别0-6）
    weights = np.ones(CONFIG['num_classes'])
    for cls, weight in zip(unique_classes, class_weights):
        weights[cls] = weight

    print("\n类别权重:")
    for i, w in enumerate(weights):
        print(f"  类别 {i}: {w:.4f}")

    return torch.FloatTensor(weights)

# === 4. 数据加载 ===
def load_data():
    print(f"正在加载 {CONFIG['input_file']}...")

    with h5py.File(CONFIG['input_file'], 'r') as f:
        X_train = f['X_train'][:]
        Y_train = f['Y_train'][:]
        X_val = f['X_val'][:]
        Y_val = f['Y_val'][:]

        print(f"\n数据集信息:")
        print(f"  训练集: X={X_train.shape}, Y={Y_train.shape}")
        print(f"  验证集: X={X_val.shape}, Y={Y_val.shape}")

    # 数据标准化
    print(f"\n正在标准化数据...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_val_scaled = scaler.transform(X_val)

    # 计算类别权重
    class_weights = compute_class_weights(Y_train)

    # 创建数据集
    train_dataset = BimanualDataset(X_train_scaled, Y_train, CONFIG['window_size'])
    val_dataset = BimanualDataset(X_val_scaled, Y_val, CONFIG['window_size'])

    train_loader = DataLoader(train_dataset, batch_size=CONFIG['batch_size'],
                              shuffle=True, num_workers=0)
    val_loader = DataLoader(val_dataset, batch_size=CONFIG['batch_size'],
                           shuffle=False, num_workers=0)

    print(f"\n实际可用样本数:")
    print(f"  训练集: {len(train_dataset)} 个窗口")
    print(f"  验证集: {len(val_dataset)} 个窗口")

    return train_loader, val_loader, scaler, class_weights

# === 5. 训练和评估 ===
def train_epoch(model, train_loader, criterion, optimizer, device):
    model.train()
    train_loss = 0.0
    correct = 0
    total = 0

    for inputs, labels in train_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        train_loss += loss.item()
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

    avg_loss = train_loss / len(train_loader)
    accuracy = 100 * correct / total
    return avg_loss, accuracy

def evaluate(model, val_loader, device):
    model.eval()
    correct = 0
    total = 0
    all_preds = []
    all_labels = []

    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

            all_preds.extend(predicted.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())

    accuracy = 100 * correct / total

    # 计算每个类别的准确率
    class_acc = {}
    for i in range(CONFIG['num_classes']):
        mask = np.array(all_labels) == i
        if mask.sum() > 0:
            class_acc[i] = np.mean(np.array(all_preds)[mask] == np.array(all_labels)[mask]) * 100
        else:
            class_acc[i] = 0.0

    return accuracy, class_acc

# === 6. 主训练循环 ===
def main():
    print("=" * 60)
    print("双手协调分类 - 改进版LSTM训练")
    print("=" * 60)

    # 加载数据
    train_loader, val_loader, scaler, class_weights = load_data()

    # 创建模型
    model = SimpleLSTM(
        CONFIG['input_dim'],
        CONFIG['hidden_dim'],
        CONFIG['num_classes']
    ).to(CONFIG['device'])

    print(f"\n模型架构:")
    print(model)
    print(f"\n设备: {CONFIG['device']}")

    # 使用加权损失函数
    class_weights = class_weights.to(CONFIG['device'])
    criterion = nn.CrossEntropyLoss(weight=class_weights)

    # 优化器和学习率调度器
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='max', factor=0.5, patience=5, verbose=True
    )

    # 训练记录
    best_val_acc = 0.0
    patience_counter = 0
    train_losses = []
    train_accs = []
    val_accs = []

    print("\n" + "=" * 60)
    print("开始训练...")
    print("=" * 60)

    for epoch in range(CONFIG['num_epochs']):
        # 训练
        train_loss, train_acc = train_epoch(
            model, train_loader, criterion, optimizer, CONFIG['device']
        )
        train_losses.append(train_loss)
        train_accs.append(train_acc)

        # 验证
        val_acc, class_acc = evaluate(model, val_loader, CONFIG['device'])
        val_accs.append(val_acc)

        # 学习率调度
        scheduler.step(val_acc)

        print(f"Epoch [{epoch+1}/{CONFIG['num_epochs']}] "
              f"Loss: {train_loss:.4f} | "
              f"Train Acc: {train_acc:.2f}% | "
              f"Val Acc: {val_acc:.2f}%")

        # 每10个epoch显示类别准确率
        if (epoch + 1) % 10 == 0:
            print("  各类别验证准确率:")
            for cls_id, acc in class_acc.items():
                print(f"    类别 {cls_id}: {acc:.2f}%")

        # 保存最佳模型
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            patience_counter = 0
            torch.save(model.state_dict(), CONFIG['save_model_path'])
            joblib.dump(scaler, CONFIG['save_scaler_path'])
            print(f"  ✓ 保存最佳模型 (Val Acc: {val_acc:.2f}%)")
        else:
            patience_counter += 1

        # 早停
        if patience_counter >= CONFIG['patience']:
            print(f"\n早停触发！已经{CONFIG['patience']}个epoch没有改进。")
            break

    print("\n" + "=" * 60)
    print(f"训练完成！最佳验证准确率: {best_val_acc:.2f}%")
    print(f"模型已保存至: {CONFIG['save_model_path']}")
    print(f"Scaler已保存至: {CONFIG['save_scaler_path']}")
    print("=" * 60)

    # 加载最佳模型并显示最终结果
    model.load_state_dict(torch.load(CONFIG['save_model_path']))
    final_val_acc, final_class_acc = evaluate(model, val_loader, CONFIG['device'])

    print("\n最终验证集结果:")
    print(f"总体准确率: {final_val_acc:.2f}%")
    print("\n各类别准确率:")
    label_names = {
        0: "No Action",
        1: "Loosely Coupled",
        2: "Unimanual Left",
        3: "Unimanual Right",
        4: "Tightly Asym (L-Dom)",
        5: "Tightly Asym (R-Dom)",
        6: "Tightly Symmetric"
    }
    for cls_id, acc in final_class_acc.items():
        print(f"  {label_names[cls_id]}: {acc:.2f}%")

    # 绘制训练曲线
    print("\n正在生成训练曲线...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(train_losses, label='Train Loss')
    axes[0].set_xlabel('Epoch')
    axes[0].set_ylabel('Loss')
    axes[0].set_title('Training Loss (Weighted)')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(train_accs, label='Train Accuracy')
    axes[1].plot(val_accs, label='Validation Accuracy')
    axes[1].set_xlabel('Epoch')
    axes[1].set_ylabel('Accuracy (%)')
    axes[1].set_title('Training and Validation Accuracy')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('training_curves_improved.png', dpi=150)
    print("训练曲线已保存至: training_curves_improved.png")

if __name__ == "__main__":
    main()
