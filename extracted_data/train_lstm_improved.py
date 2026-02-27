import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import h5py
import numpy as np
from sklearn.utils.class_weight import compute_class_weight

CONFIG = {
    'input_file': '219merged_dataset_3d.h5',
    'input_dim': 5,        # 
    'hidden_dim': 64,      # 
    'layer_dim': 1,         # 2 层 LSTM
    'dropout': 0.3,
    'num_classes': 7,      # 
    'batch_size': 128,
    'learning_rate': 0.001,
    'num_epochs': 50,
    'device': 'cuda' if torch.cuda.is_available() else 'cpu'
}

class BinaryDataset(Dataset):
    def __init__(self, X, Y):
        self.X = torch.FloatTensor(X)
        self.Y = torch.LongTensor(Y)
    def __len__(self): return len(self.X)
    def __getitem__(self, idx): return self.X[idx], self.Y[idx]

class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes, num_layers):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True, num_layers=num_layers)
        self.fc = nn.Linear(hidden_dim, num_classes)
    def forward(self, x):
        _, (h_n, _) = self.lstm(x) # 也可以取最后一个h_n
        return self.fc(h_n[-1])

def main():
    # 1. 加载数据
    with h5py.File(CONFIG['input_file'], 'r') as f:
        X_train, Y_train = f['X_train'][:], f['Y_train'][:]
        X_val, Y_val = f['X_val'][:], f['Y_val'][:]

    # 2. 计算权重 (处理“不抓取”多于“抓取”的情况)
    # 虽然是二元，但如果样本不均衡，这步依然至关重要
    #weights = compute_class_weight('balanced', classes=np.array([0, 1]), y=Y_train)
    #class_weights = torch.FloatTensor(weights).to(CONFIG['device'])

    classes = np.unique(Y_train)
    weights = compute_class_weight('balanced', classes=classes, y=Y_train)
    class_weights = torch.FloatTensor(weights).to(CONFIG['device'])
    CONFIG['num_classes'] = len(classes)  # 同步输出层大小


    train_loader = DataLoader(BinaryDataset(X_train, Y_train), batch_size=CONFIG['batch_size'], shuffle=True)
    val_loader = DataLoader(BinaryDataset(X_val, Y_val), batch_size=CONFIG['batch_size'])

    # 3. 初始化
    model = SimpleLSTM(CONFIG['input_dim'], CONFIG['hidden_dim'], CONFIG['num_classes'], CONFIG['layer_dim']).to(CONFIG['device'])
    criterion = nn.CrossEntropyLoss(weight=class_weights) # 使用加权损失
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])

    # 4. 训练
    print("开始分类训练...")
    for epoch in range(CONFIG['num_epochs']):
        model.train()
        l_sum = 0
        for bx, by in train_loader:
            bx, by = bx.to(CONFIG['device']), by.to(CONFIG['device'])
            optimizer.zero_grad()
            output = model(bx)
            loss = criterion(output, by)
            loss.backward()
            optimizer.step()
            l_sum += loss.item()
        
        # 验证
        model.eval()
        correct = 0
        with torch.no_grad():
            for vx, vy in val_loader:
                vx, vy = vx.to(CONFIG['device']), vy.to(CONFIG['device'])
                pred = model(vx).argmax(1)
                correct += (pred == vy).sum().item()
        
        acc = 100 * correct / len(Y_val)
        print(f"Epoch {epoch+1}, Loss: {l_sum/len(train_loader):.4f}, Val Acc: {acc:.2f}%")

    torch.save(model.state_dict(), "219grasp_model.pth")
    print("模型已保存。")

if __name__ == "__main__":
    main()