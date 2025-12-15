import pickle

metadata_file = "cam_0_rgb_video.metadata"   # 替换成你的文件路径

with open(metadata_file, "rb") as f:
    metadata = pickle.load(f)

print("Metadata keys:", metadata.keys())
print("Example metadata:", metadata)
