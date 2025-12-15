"""
使用合并数据集训练LSTM模型
- 训练集: demo11 + demo13
- 验证集: demo12
- 数据已经裁剪了前50帧
"""
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import h5py
import numpy as np
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import joblib

# === 配置 ===
CONFIG = {
    'input_file': 'merged_dataset.h5',
    'window_size': 50,      # LSTM窗口大小
    'input_dim': 8,         # 特征维度
    'hidden_dim': 128,      # LSTM隐藏层维度
    'num_classes': 7,       # 类别数量 (0-6)
    'batch_size': 32,
    'learning_rate': 0.001,
    'num_epochs': 50,
    'device': 'cuda' if torch.cuda.is_available() else 'cpu',
    'save_model_path': 'best_lstm_merged_model.pth',
    'save_scaler_path': 'scaler_merged.pkl'
}

# === 1. 数据集类 ===
class BimanualDataset(Dataset):
    def __init__(self, X, Y, window_size):
        """
        X: (N, 8) 特征数据
        Y: (N, ) 标签数据
        window_size: LSTM序列窗口大小
        """
        self.X = torch.FloatTensor(X)
        self.Y = torch.LongTensor(Y)
        self.window_size = window_size

    def __len__(self):
        # 因为需要window_size个历史帧，所以总样本数减少
        return len(self.X) - self.window_size + 1

    def __getitem__(self, idx):
        # 获取窗口序列: [idx, idx+window_size)
        x_seq = self.X[idx : idx + self.window_size]

        # 标签: 预测窗口最后一帧的标签
        y_label = self.Y[idx + self.window_size - 1]

        return x_seq, y_label

# === 2. LSTM模型 ===
class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes):
        super(SimpleLSTM, self).__init__()

        # LSTM层
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True)

        # 全连接分类层
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        # x shape: (batch, window_size, input_dim)

        # LSTM输出
        out, _ = self.lstm(x)

        # 只取最后一个时间步的输出
        last_step_out = out[:, -1, :]  # (batch, hidden_dim)

        # 分类
        logits = self.fc(last_step_out)  # (batch, num_classes)
        return logits

# === 3. 数据加载 ===
def load_data():
    """
    加载合并后的数据集
    """
    print(f"正在加载 {CONFIG['input_file']}...")

    with h5py.File(CONFIG['input_file'], 'r') as f:
        X_train = f['X_train'][:]
        Y_train = f['Y_train'][:]
        X_val = f['X_val'][:]
        Y_val = f['Y_val'][:]

        print(f"\n数据集信息:")
        print(f"  训练集: X={X_train.shape}, Y={Y_train.shape}")
        print(f"  验证集: X={X_val.shape}, Y={Y_val.shape}")

        # 显示元数据
        print(f"\n元数据:")
        for key, value in f.attrs.items():
            print(f"  {key}: {value}")

    # 数据标准化 - 重要！
    # 只在训练集上fit，然后transform训练集和验证集
    print(f"\n正在标准化数据...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_val_scaled = scaler.transform(X_val)

    # 创建数据集和DataLoader
    train_dataset = BimanualDataset(X_train_scaled, Y_train, CONFIG['window_size'])
    val_dataset = BimanualDataset(X_val_scaled, Y_val, CONFIG['window_size'])

    train_loader = DataLoader(
        train_dataset,
        batch_size=CONFIG['batch_size'],
        shuffle=True,
        num_workers=0  # 设置为0避免多进程问题
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=CONFIG['batch_size'],
        shuffle=False,
        num_workers=0
    )

    print(f"\n实际可用样本数 (考虑窗口):")
    print(f"  训练集: {len(train_dataset)} 个窗口")
    print(f"  验证集: {len(val_dataset)} 个窗口")

    return train_loader, val_loader, scaler

# === 4. 训练和评估 ===
def train_epoch(model, train_loader, criterion, optimizer, device):
    """训练一个epoch"""
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
    """评估模型"""
    model.eval()
    correct = 0
    total = 0

    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

    accuracy = 100 * correct / total
    return accuracy

# === 5. 主训练循环 ===
def main():
    print("=" * 60)
    print("双手协调分类 - LSTM训练")
    print("=" * 60)

    # 加载数据
    train_loader, val_loader, scaler = load_data()

    # 创建模型
    model = SimpleLSTM(
        CONFIG['input_dim'],
        CONFIG['hidden_dim'],
        CONFIG['num_classes']
    ).to(CONFIG['device'])

    print(f"\n模型架构:")
    print(model)
    print(f"\n设备: {CONFIG['device']}")

    # 损失函数和优化器
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])

    # 训练记录
    best_val_acc = 0.0
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
        val_acc = evaluate(model, val_loader, CONFIG['device'])
        val_accs.append(val_acc)

        print(f"Epoch [{epoch+1}/{CONFIG['num_epochs']}] "
              f"Loss: {train_loss:.4f} | "
              f"Train Acc: {train_acc:.2f}% | "
              f"Val Acc: {val_acc:.2f}%")

        # 保存最佳模型
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), CONFIG['save_model_path'])
            joblib.dump(scaler, CONFIG['save_scaler_path'])
            print(f"  ✓ 保存最佳模型 (Val Acc: {val_acc:.2f}%)")

    print("\n" + "=" * 60)
    print(f"训练完成！最佳验证准确率: {best_val_acc:.2f}%")
    print(f"模型已保存至: {CONFIG['save_model_path']}")
    print(f"Scaler已保存至: {CONFIG['save_scaler_path']}")
    print("=" * 60)

    # 绘制训练曲线
    print("\n正在生成训练曲线...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Loss曲线
    axes[0].plot(train_losses, label='Train Loss')
    axes[0].set_xlabel('Epoch')
    axes[0].set_ylabel('Loss')
    axes[0].set_title('Training Loss')
    axes[0].legend()
    axes[0].grid(True)

    # Accuracy曲线
    axes[1].plot(train_accs, label='Train Accuracy')
    axes[1].plot(val_accs, label='Validation Accuracy')
    axes[1].set_xlabel('Epoch')
    axes[1].set_ylabel('Accuracy (%)')
    axes[1].set_title('Training and Validation Accuracy')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('training_curves_merged.png', dpi=150)
    print("训练曲线已保存至: training_curves_merged.png")

if __name__ == "__main__":
    main()
