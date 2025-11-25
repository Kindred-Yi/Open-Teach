#!/usr/bin/env python3
import pickle
import datetime
import sys

def read_metadata(metadata_file):
    """读取并显示metadata文件内容"""
    try:
        with open(metadata_file, 'rb') as f:
            metadata = pickle.load(f)

        print("=== Metadata Information ===")
        print(f"File name: {metadata.get('file_name', 'N/A')}")
        print(f"Number of datapoints: {metadata.get('num_datapoints', 'N/A')}")
        print(f"Record duration: {metadata.get('record_duration', 'N/A'):.2f} seconds")
        print(f"Record frequency: {metadata.get('record_frequency', 'N/A'):.2f} Hz")
        print(f"Recorder IP: {metadata.get('recorder_ip_address', 'N/A')}")
        print(f"Image stream port: {metadata.get('recorder_image_stream_port', 'N/A')}")

        # 转换时间戳
        if 'record_start_time' in metadata:
            start_time = datetime.datetime.fromtimestamp(metadata['record_start_time'])
            print(f"Start time: {start_time}")

        if 'record_end_time' in metadata:
            end_time = datetime.datetime.fromtimestamp(metadata['record_end_time'])
            print(f"End time: {end_time}")

        # 显示时间戳信息
        if 'timestamps' in metadata:
            timestamps = metadata['timestamps']
            print(f"\nTimestamp information:")
            print(f"Total timestamps: {len(timestamps)}")

            try:
                # 尝试作为秒级时间戳
                print(f"First timestamp: {datetime.datetime.fromtimestamp(timestamps[0])}")
                print(f"Last timestamp: {datetime.datetime.fromtimestamp(timestamps[-1])}")
            except (ValueError, OSError):
                try:
                    # 尝试作为毫秒级时间戳
                    print(f"First timestamp: {datetime.datetime.fromtimestamp(timestamps[0]/1000)}")
                    print(f"Last timestamp: {datetime.datetime.fromtimestamp(timestamps[-1]/1000)}")
                except (ValueError, OSError) as e:
                    print(f"Cannot convert timestamps to datetime: {e}")
                    print(f"First timestamp (raw): {timestamps[0]}")
                    print(f"Last timestamp (raw): {timestamps[-1]}")

            # 计算时间间隔 (转换为秒)
            if len(timestamps) > 1:
                # 如果是毫秒级时间戳，转换为秒
                ts_in_seconds = [ts/1000 if ts > 1e10 else ts for ts in timestamps]
                intervals = [ts_in_seconds[i] - ts_in_seconds[i-1] for i in range(1, min(10, len(ts_in_seconds)))]
                avg_interval = sum(intervals) / len(intervals)
                print(f"Average interval between frames: {avg_interval:.4f} seconds")
                print(f"Average FPS: {1/avg_interval:.2f}")

                # 显示前几个时间戳的相对时间
                print(f"\nFirst 5 timestamps (relative to start):")
                for i in range(min(5, len(ts_in_seconds))):
                    rel_time = ts_in_seconds[i] - ts_in_seconds[0]
                    print(f"  Frame {i}: +{rel_time:.4f}s")

        print(f"\nAll metadata keys: {list(metadata.keys())}")

    except Exception as e:
        print(f"Error reading metadata: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python read_metadata.py <metadata_file>")
        sys.exit(1)

    metadata_file = sys.argv[1]
    read_metadata(metadata_file)