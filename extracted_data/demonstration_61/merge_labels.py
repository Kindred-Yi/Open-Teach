import json
import h5py
import numpy as np

# === 配置 ===
JSON_PATH = "project-1-at-2026-02-07-18-32-ea52fa4a.json"  # 你的 Label Studio 导出文件
H5_PATH = "aligned_data.h5"     # 之前对齐好的 H5 文件
TOTAL_FRAMES = 4942             # 视频总帧数 (根据你之前的描述)

# 定义类别映射 (String -> ID)
# 建议：No Action 设为 0，其他任意
LABEL_MAPPING = {
    "No Action": 0,
    "Loosely Coupled": 1,
    "Unimanual Left": 2,
    "Unimanual Right": 3,
    "Tightly Asym (L-Dom)": 4,
    "Tightly Asym (R-Dom)": 5,
    "Tightly Symmetric":6
}

def main():
    print(f"正在读取标注文件: {JSON_PATH}")
    with open(JSON_PATH, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 1. 初始化全 0 数组 (默认是 No Action)
    # 形状: (5061,)
    frame_labels = np.zeros(TOTAL_FRAMES, dtype=np.int64)
    
    # 2. 解析 JSON 并填充数组
    # Label Studio 的结构通常是 data[0]['annotations'][0]['result']
    results = data[1]['annotations'][0]['result']
    
    print("开始转换标签...")
    for segment in results:
        # 获取标签名称
        # 注意：timelinelabels 是一个列表，通常只有一个元素
        label_name = segment['value']['timelinelabels'][0]
        
        # 获取起止帧 (Label Studio 导出通常是 1-based index)
        r = segment['value']['ranges'][0]
        start_frame = int(r['start'])
        end_frame = int(r['end'])
        
        if label_name in LABEL_MAPPING:
            label_id = LABEL_MAPPING[label_name]
            
            # 转换为 Python 的 0-based index
            # start-1 : end (因为 end 也是包含的，所以切片时不需要 -1，Python切片是左闭右开)
            # 例如: 1-10帧 -> index 0-9 -> slice 0:10
            idx_start = max(0, start_frame - 1)
            idx_end = min(TOTAL_FRAMES, end_frame)
            
            frame_labels[idx_start : idx_end] = label_id
            
            print(f"  - 帧 {start_frame:04d}-{end_frame:04d} -> {label_name} (ID: {label_id})")
        else:
            print(f"  [警告] 未知标签: {label_name}")

    # 3. 写入 H5 文件
    print(f"\n正在写入 H5 文件: {H5_PATH} ...")
    with h5py.File(H5_PATH, 'r+') as f:  # 'r+' 模式允许读写已存在的文件
        # 如果已经存在旧标签，先删除
        if 'coordination_labels' in f:
            del f['coordination_labels']
        
        # 创建新数据集
        dset = f.create_dataset('coordination_labels', data=frame_labels)
        
        # 将映射关系保存为属性，防止以后忘了 0/1/2 代表什么
        # H5 属性不支持字典，转成 JSON 字符串存进去
        f.attrs['label_mapping'] = json.dumps(LABEL_MAPPING, ensure_ascii=False)
        
        print(f"成功！已添加数据集 'coordination_labels', shape={frame_labels.shape}")
        print(f"映射关系已保存至文件属性 'label_mapping'")

    # 4. 验证一下
    with h5py.File(H5_PATH, 'r') as f:
        print("\n=== 验证文件内容 ===")
        print("Keys:", list(f.keys()))
        print("Labels Sample (前10帧):", f['coordination_labels'][:10])
        print("Mapping Attr:", f.attrs['label_mapping'])

if __name__ == "__main__":
    main()
