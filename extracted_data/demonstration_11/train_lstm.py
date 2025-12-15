import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import h5py
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt


CONFIG = {
    'input_file': 'lstm_danny_features.h5',
    'window_size': 50,      # 
    'input_dim': 8,         # 
    'hidden_dim': 128,      # 
    'num_classes': 6,       # 0-5 
    'batch_size': 32,
    'learning_rate': 0.001,
    'num_epochs': 50,
    'device': 'cuda' if torch.cuda.is_available() else 'cpu'
}

class BimanualDataset(Dataset):
    def __init__(self, X, Y, window_size):
        """
        X: (N, 8) 
        Y: (N, ) 
        window_size: LSTM sequence
        """
        self.X = torch.FloatTensor(X)
        self.Y = torch.LongTensor(Y)
        self.window_size = window_size

    def __len__(self):
        return len(self.X) - self.window_size

    def __getitem__(self, idx):
        # shape: (window_size, 8)
        x_seq = self.X[idx : idx + self.window_size]
        
        # 标签: 我们想预测这个窗口“最后那一帧”发生的事情
        # 对应 index: idx + window_size - 1
        y_label = self.Y[idx + self.window_size - 1]
        
        return x_seq, y_label

# === 2. 定义神经网络 (Danny's Architecture) ===
class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes):
        super(SimpleLSTM, self).__init__()
        
        # 简单的单层 LSTM
        # batch_first=True -> 输入形状 (Batch, Seq, Feature)
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True)
        
        # 全连接层分类器
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        # x shape: (batch, window_size, 8)
        
        # LSTM 输出
        # out shape: (batch, window_size, hidden_dim)
        # _ (hn, cn): 隐藏状态 (这里不需要)
        out, _ = self.lstm(x)
        
        # 我们只关心序列中**最后一个时间步**的输出
        # 因为我们是用过去的历史来分类“当前”的状态
        last_step_out = out[:, -1, :]  # Shape: (batch, hidden_dim)
        
        # 分类
        logits = self.fc(last_step_out) # Shape: (batch, num_classes)
        return logits

# === 3. 数据预处理与加载 ===
def load_and_process_data():
    print(f"正在加载 {CONFIG['input_file']}...")
    with h5py.File(CONFIG['input_file'], 'r') as f:
        X_raw = f['X'][:]
        Y_raw = f['Y'][:]

    print(f"原始数据形状: X={X_raw.shape}, Y={Y_raw.shape}")

    # --- 重要: 数据标准化 (Standardization) ---
    # 神经网络对数据的尺度非常敏感。
    # 位置是米(0.5左右)，速度可能是(0.01)，Gripper变化率极小。
    # 不做标准化，模型很难收敛。
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_raw)
    
    # 划分训练集和验证集 (80% 训练, 20% 验证)
    # shuffle=False 是为了保持时间序列的连续性吗？
    # 对于测试模型泛化能力，通常建议 shuffle=True 打乱片段，
    # 但如果为了展示严谨的时序分割，应该按时间切分。
    # 这里我们简单随机打乱划分，因为我们已经切成了 window 样本。
    X_train, X_val, y_train, y_val = train_test_split(
        X_scaled, Y_raw, test_size=0.2, random_state=42, shuffle=False
    )
    
    train_dataset = BimanualDataset(X_train, y_train, CONFIG['window_size'])
    val_dataset = BimanualDataset(X_val, y_val, CONFIG['window_size'])
    
    train_loader = DataLoader(train_dataset, batch_size=CONFIG['batch_size'], shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=CONFIG['batch_size'], shuffle=False)
    
    return train_loader, val_loader, scaler

# === 4. 训练循环 ===
def main():
    train_loader, val_loader, scaler = load_and_process_data()
    
    model = SimpleLSTM(CONFIG['input_dim'], CONFIG['hidden_dim'], CONFIG['num_classes']).to(CONFIG['device'])
    
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])
    
    print(f"模型架构:\n{model}")
    print(f"开始在 {CONFIG['device']} 上训练...")

    best_val_acc = 0.0
    loss_history = []

    for epoch in range(CONFIG['num_epochs']):
        # --- Training ---
        model.train()
        train_loss = 0.0
        correct = 0
        total = 0
        
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(CONFIG['device']), labels.to(CONFIG['device'])
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
            
        train_acc = 100 * correct / total
        avg_loss = train_loss / len(train_loader)
        loss_history.append(avg_loss)

        # --- Validation ---
        model.eval()
        val_correct = 0
        val_total = 0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(CONFIG['device']), labels.to(CONFIG['device'])
                outputs = model(inputs)
                _, predicted = torch.max(outputs.data, 1)
                val_total += labels.size(0)
                val_correct += (predicted == labels).sum().item()
        
        val_acc = 100 * val_correct / val_total
        
        print(f"Epoch [{epoch+1}/{CONFIG['num_epochs']}] "
              f"Loss: {avg_loss:.4f} | Train Acc: {train_acc:.2f}% | Val Acc: {val_acc:.2f}%")
        
        # 保存最佳模型
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), 'best_lstm_model.pth')
            # 同时也保存 scaler，因为推理时需要用到它来归一化新数据
            import joblib
            joblib.dump(scaler, 'scaler.pkl')

    print(f"训练结束。最佳验证集准确率: {best_val_acc:.2f}%")
    print("模型已保存为 'best_lstm_model.pth'")
    
    # 绘制 Loss 曲线
    plt.plot(loss_history)
    plt.title('Training Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.savefig('loss_curve.png')
    # plt.show()

if __name__ == "__main__":
    main()