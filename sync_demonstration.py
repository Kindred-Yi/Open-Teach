#!/usr/bin/env python3
"""
Utility to align recorded robot states with RGB frames for a demonstration run.

Example:
    python sync_demonstration.py --demo 11
"""
import argparse
import csv
import glob
import os
import pickle
from typing import Dict, List, Tuple

import h5py
import numpy as np

SKIP_DATASETS = {
    'file_name',
    'num_datapoints',
    'record_start_time',
    'record_end_time',
    'record_duration',
    'record_frequency',
    'timestamps',
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Align robot joint states and RGB frames using timestamps.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--demo',
        type=int,
        help='Demonstration index (e.g. 11 for demonstration_11).',
    )
    parser.add_argument(
        '--storage-path',
        default='extracted_data',
        help='Root folder that holds demonstration_<N> directories.',
    )
    parser.add_argument(
        '--demo-path',
        help='Explicit path to a demonstration folder. Overrides --demo/--storage-path.',
    )
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera index to align against (cam_<camera>_rgb_video.*).',
    )
    parser.add_argument(
        '--output-dir',
        help='Directory to place synchronized artifacts. Defaults to <demo_path>/synced_cam_<camera>.',
    )
    parser.add_argument(
        '--dump-frames',
        action='store_true',
        help='Decode the RGB video and store each frame as an image.',
    )
    parser.add_argument(
        '--image-ext',
        default='jpg',
        help='Extension used when dumping frames (requires --dump-frames).',
    )
    return parser.parse_args()


def ensure_seconds(timestamps: np.ndarray) -> np.ndarray:
    """Convert millisecond timestamps to seconds if needed."""
    ts = np.asarray(timestamps, dtype=np.float64)
    if ts.size == 0:
        raise ValueError('Timestamp array is empty.')

    if np.nanmax(ts) > 1e11:
        ts = ts / 1e3
    return ts


def load_camera_metadata(metadata_path: str) -> Tuple[Dict, np.ndarray]:
    with open(metadata_path, 'rb') as f:
        metadata = pickle.load(f)

    if 'timestamps' not in metadata:
        raise KeyError(f'No timestamps found in {metadata_path}.')
    timestamps = ensure_seconds(np.asarray(metadata['timestamps'], dtype=np.float64))
    return metadata, timestamps


def gather_robot_files(demo_path: str) -> List[str]:
    pattern = os.path.join(demo_path, '*.h5')
    files = []
    for path in glob.glob(pattern):
        name = os.path.basename(path)
        if name.startswith('cam_'):
            continue
        files.append(path)
    if not files:
        raise FileNotFoundError(f'No robot *.h5 files found in {demo_path}.')
    return sorted(files)


def _sort_and_unique(ts: np.ndarray, values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    order = np.argsort(ts)
    ts_sorted = ts[order]
    values_sorted = values[order]
    if ts_sorted.size == 0:
        return ts_sorted, values_sorted
    unique_mask = np.ones_like(ts_sorted, dtype=bool)
    unique_mask[1:] = np.diff(ts_sorted) > 0
    return ts_sorted[unique_mask], values_sorted[unique_mask]


def interpolate_values(source_ts: np.ndarray, values: np.ndarray, target_ts: np.ndarray) -> np.ndarray:
    if values.shape[0] != source_ts.shape[0]:
        raise ValueError('Mismatched timestamps and value shapes for interpolation.')

    clipped_targets = np.clip(target_ts, source_ts[0], source_ts[-1])
    flat_values = values.reshape(values.shape[0], -1)
    interpolated = np.empty((target_ts.shape[0], flat_values.shape[1]), dtype=np.float32)
    for idx in range(flat_values.shape[1]):
        interpolated[:, idx] = np.interp(
            clipped_targets, source_ts, flat_values[:, idx].astype(np.float64)
        )
    return interpolated.reshape(target_ts.shape[0], *values.shape[1:])


def resample_robot_file(file_path: str, target_ts: np.ndarray) -> Dict[str, np.ndarray]:
    aligned_data: Dict[str, np.ndarray] = {}
    with h5py.File(file_path, 'r') as source:
        if 'timestamps' not in source:
            raise KeyError(f'{file_path} does not contain a timestamps dataset.')
        timestamps = np.asarray(source['timestamps'][:], dtype=np.float64)
        if timestamps.size == 0:
            return aligned_data

        for key in source.keys():
            if key in SKIP_DATASETS:
                continue
            dataset = source[key]
            if dataset.shape == () or dataset.shape[0] != timestamps.shape[0]:
                continue
            if dataset.dtype.kind not in ('f', 'i'):
                continue

            values = np.asarray(dataset[:], dtype=np.float64)
            clean_ts, clean_values = _sort_and_unique(timestamps, values)
            if clean_ts.size < 2:
                continue

            aligned_data[key] = interpolate_values(clean_ts, clean_values, target_ts)
    return aligned_data


def extract_frames(video_path: str, frame_count: int, output_dir: str, image_ext: str) -> List[str]:
    import cv2

    os.makedirs(output_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    frame_paths: List[str] = []
    idx = 0
    while idx < frame_count:
        ok, frame = cap.read()
        if not ok:
            break
        frame_path = os.path.join(output_dir, f'frame_{idx:06d}.{image_ext}')
        if not cv2.imwrite(frame_path, frame):
            raise RuntimeError(f'Failed to write {frame_path}')
        frame_paths.append(frame_path)
        idx += 1
    cap.release()

    if idx != frame_count:
        raise RuntimeError(
            f'Expected {frame_count} frames from {video_path}, but decoded {idx}.'
        )
    return frame_paths


def write_outputs(
    output_dir: str,
    video_path: str,
    metadata: Dict,
    camera_ts: np.ndarray,
    robot_data: Dict[str, Dict[str, np.ndarray]],
    original_paths: Dict[str, str],
    frame_paths: List[str],
) -> None:
    os.makedirs(output_dir, exist_ok=True)
    relative_video = os.path.relpath(video_path, output_dir)
    synced_file = os.path.join(output_dir, 'synced_data.h5')

    with h5py.File(synced_file, 'w') as sink:
        cam_group = sink.create_group('camera')
        cam_group.create_dataset(
            'timestamps', data=camera_ts, compression='gzip', compression_opts=6
        )
        cam_group.create_dataset(
            'relative_time',
            data=camera_ts - camera_ts[0],
            compression='gzip',
            compression_opts=6,
        )
        cam_group.create_dataset(
            'frame_indices',
            data=np.arange(camera_ts.shape[0], dtype=np.int32),
            compression='gzip',
            compression_opts=6,
        )
        cam_group.attrs['video_path'] = relative_video
        for key, value in metadata.items():
            if key == 'timestamps':
                continue
            try:
                cam_group.attrs[key] = value
            except TypeError:
                continue

        robot_group = sink.create_group('robot')
        for name, datasets in robot_data.items():
            sensor_group = robot_group.create_group(name)
            sensor_group.attrs['source_file'] = os.path.relpath(
                original_paths[name], output_dir
            )
            for dataset_name, data in datasets.items():
                sensor_group.create_dataset(
                    dataset_name,
                    data=data,
                    compression='gzip',
                    compression_opts=6,
                )

        if frame_paths:
            frame_refs = cam_group.create_dataset(
                'frame_paths',
                shape=(len(frame_paths),),
                dtype=h5py.string_dtype(encoding='utf-8'),
            )
            for idx, path in enumerate(frame_paths):
                frame_refs[idx] = os.path.relpath(path, output_dir)

    csv_path = os.path.join(output_dir, 'synced_index.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        header = ['frame_idx', 'timestamp', 'relative_time']
        if frame_paths:
            header.append('frame_path')
        writer.writerow(header)
        for idx, timestamp in enumerate(camera_ts):
            row = [idx, timestamp, timestamp - camera_ts[0]]
            if frame_paths:
                row.append(os.path.relpath(frame_paths[idx], output_dir))
            writer.writerow(row)

    print(f'Stored synchronized arrays in {synced_file}')
    print(f'Index CSV saved to {csv_path}')
    if frame_paths:
        print(f'RGB frames saved under {os.path.relpath(os.path.dirname(frame_paths[0]), output_dir)}')


def main() -> None:
    args = parse_args()
    if not args.demo_path and args.demo is None:
        raise ValueError('Provide either --demo or --demo-path.')

    demo_path = args.demo_path
    if demo_path is None:
        demo_path = os.path.join(args.storage_path, f'demonstration_{args.demo}')
    demo_path = os.path.abspath(demo_path)
    if not os.path.isdir(demo_path):
        raise FileNotFoundError(f'Demonstration folder {demo_path} does not exist.')

    metadata_path = os.path.join(demo_path, f'cam_{args.camera}_rgb_video.metadata')
    video_path = os.path.join(demo_path, f'cam_{args.camera}_rgb_video.avi')
    if not os.path.isfile(metadata_path):
        raise FileNotFoundError(f'Could not find metadata file: {metadata_path}')
    if not os.path.isfile(video_path):
        raise FileNotFoundError(f'Could not find RGB video: {video_path}')

    metadata, camera_ts = load_camera_metadata(metadata_path)
    robot_files = gather_robot_files(demo_path)
    robot_data: Dict[str, Dict[str, np.ndarray]] = {}
    for file_path in robot_files:
        name = os.path.splitext(os.path.basename(file_path))[0]
        aligned = resample_robot_file(file_path, camera_ts)
        if aligned:
            robot_data[name] = aligned
            print(f'Aligned datasets from {file_path}')
        else:
            print(f'No alignable datasets found in {file_path}, skipping.')

    if not robot_data:
        raise RuntimeError('No robot datasets were aligned successfully.')

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.join(demo_path, f'synced_cam_{args.camera}')
    output_dir = os.path.abspath(output_dir)

    frame_paths: List[str] = []
    if args.dump_frames:
        frames_dir = os.path.join(output_dir, 'rgb_frames')
        frame_paths = extract_frames(video_path, camera_ts.shape[0], frames_dir, args.image_ext)

    write_outputs(
        output_dir=output_dir,
        video_path=video_path,
        metadata=metadata,
        camera_ts=camera_ts,
        robot_data=robot_data,
        original_paths={os.path.splitext(os.path.basename(p))[0]: p for p in robot_files},
        frame_paths=frame_paths,
    )


if __name__ == '__main__':
    main()
