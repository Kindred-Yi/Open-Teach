import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import h5py
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.utils.class_weight import compute_class_weight
from sklearn.metrics import f1_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import seaborn as sns
import joblib

# === 配置参数 ===
CONFIG = {
    'input_file': 'actu_lstm_danny_features.h5', # 使用实际位置数据
    'window_size': 50,
    'input_dim': 8,
    'hidden_dim': 128,
    'num_classes': 7,
    'batch_size': 64,
    'learning_rate': 0.001,
    'num_epochs': 100,
    'patience': 20,
    'split_idx': 5000,
    'device': 'cuda' if torch.cuda.is_available() else 'cpu',
    'save_model_path': 'best_lstm_model.pth',
}

# === 1. 数据集加载与预处理 ===
class BimanualDataset(Dataset):
    def __init__(self, X, Y, window_size):
        self.X = torch.FloatTensor(X)
        self.Y = torch.LongTensor(Y)
        self.window_size = window_size

    def __len__(self):
        # 窗口化后的有效长度
        return len(self.X) - self.window_size + 1

    def __getitem__(self, idx):
        # 获取从 idx 到 idx + window_size 的特征序列
        x_seq = self.X[idx : idx + self.window_size]
        # 标签取窗口最后一帧对应的标签
        y_label = self.Y[idx + self.window_size - 1]
        return x_seq, y_label

def prepare_data():
    with h5py.File(CONFIG['input_file'], 'r') as f:
        X = f['X'][:]
        Y = f['Y'][:]
    
    # 分割原始序列
    # 训练集: 0-4999
    X_train_raw = X[:CONFIG['split_idx']]
    Y_train_raw = Y[:CONFIG['split_idx']]
    
    # 验证集: 5000-结尾
    X_val_raw = X[CONFIG['split_idx']:]
    Y_val_raw = Y[CONFIG['split_idx']:]
    
    # 归一化 (在训练集上Fit)
    scaler = StandardScaler()
    X_train_raw = scaler.fit_transform(X_train_raw)
    X_val_raw = scaler.transform(X_val_raw)
    joblib.dump(scaler, 'scaler.pkl')

    # 构建带窗口的数据集
    train_dataset = BimanualDataset(X_train_raw, Y_train_raw, CONFIG['window_size'])
    val_dataset = BimanualDataset(X_val_raw, Y_val_raw, CONFIG['window_size'])
    
    print(f"训练样本数 (窗口化后): {len(train_dataset)}")
    print(f"验证样本数 (窗口化后): {len(val_dataset)}")
    
    return train_dataset, val_dataset, Y_train_raw

# === 2. 模型定义 ===
class ImprovedLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes):
        super(ImprovedLSTM, self).__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers=2, 
                            batch_first=True, dropout=0.2)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 64),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64, num_classes)
        )

    def forward(self, x):
        # x shape: (batch, seq_len, input_dim)
        out, _ = self.lstm(x)
        # 取最后一个时间步的输出
        last_step_out = out[:, -1, :]
        return self.fc(last_step_out)

# === 3. 训练函数 ===
def train():
    train_ds, val_ds, Y_train_raw = prepare_data()
    
    train_loader = DataLoader(train_ds, batch_size=CONFIG['batch_size'], shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=CONFIG['batch_size'], shuffle=False)

    # 计算类别权重处理不平衡
    class_weights = compute_class_weight('balanced', classes=np.unique(Y_train_raw), y=Y_train_raw)
    weights_tensor = torch.FloatTensor(class_weights).to(CONFIG['device'])

    model = ImprovedLSTM(CONFIG['input_dim'], CONFIG['hidden_dim'], CONFIG['num_classes']).to(CONFIG['device'])
    criterion = nn.CrossEntropyLoss(weight=weights_tensor)
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])

    best_val_loss = float('inf')
    counter = 0

    for epoch in range(CONFIG['num_epochs']):
        # Training
        model.train()
        train_loss = 0
        for x_batch, y_batch in train_loader:
            x_batch, y_batch = x_batch.to(CONFIG['device']), y_batch.to(CONFIG['device'])
            optimizer.zero_grad()
            outputs = model(x_batch)
            loss = criterion(outputs, y_batch)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        # Validation
        model.eval()
        val_loss = 0
        all_preds = []
        all_labels = []
        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch, y_batch = x_batch.to(CONFIG['device']), y_batch.to(CONFIG['device'])
                outputs = model(x_batch)
                loss = criterion(outputs, y_batch)
                val_loss += loss.item()
                preds = torch.argmax(outputs, dim=1)
                all_preds.extend(preds.cpu().numpy())
                all_labels.extend(y_batch.cpu().numpy())

        avg_train_loss = train_loss / len(train_loader)
        avg_val_loss = val_loss / len(val_loader)
        f1 = f1_score(all_labels, all_preds, average='macro')

        print(f"Epoch {epoch+1}/{CONFIG['num_epochs']} - Train Loss: {avg_train_loss:.4f} - Val Loss: {avg_val_loss:.4f} - Val F1: {f1:.4f}")

        # 早停逻辑
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save(model.state_dict(), CONFIG['save_model_path'])
            counter = 0
        else:
            counter += 1
            if counter >= CONFIG['patience']:
                print("Early stopping triggered.")
                break

    print("训练完成，最佳模型已保存。")

if __name__ == "__main__":
    train()