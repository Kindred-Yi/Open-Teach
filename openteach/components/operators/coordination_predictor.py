"""
CoordinationPredictor: Real-time bimanual coordination mode classifier.

Data pipeline matching training EXACTLY:

  Training:
    ros2 topics → DexArmControl subscribes → TesolloHand.get_full_controller_state()
    → RobotInformationRecord saves H5 @ 60Hz
    → align_data.py interpolates to 30fps (camera rate)
    → feature_generate.py computes 25-dim features
    → merge_datasets.py creates 50-frame windows → train LSTM

  Inference (this):
    ros2 topics → single Node subscribes (same topics as DexArmControl)
    → assemble state dict (same logic as TesolloHand.get_full_controller_state())
    → sample @ 30Hz (matching aligned training data)
    → same feature computation as feature_generate.py
    → 50-frame sliding window = 1.67s → LSTM inference
    → publish /coordination_mode

ROS2 subscriptions (per hand):
    /{hand}/dg3f_b_controller/controller_state  (actual + commanded + error)
    /{hand}/vr_sent_command                     (VR commanded positions)

ZMQ subscriptions (direct from transform process, bypass ROS2):
    transformed_hand_frame on right port        (wrist position + orientation)
    transformed_hand_frame on left port         (wrist position + orientation)

Labels:
    0 = No Action,  1 = Loosely Coupled,  2 = Unimanual Left,
    3 = Unimanual Right,  4 = Tightly Asym (L-Dom),
    5 = Tightly Asym (R-Dom),  6 = Tightly Symmetric
"""

import os
import time
import numpy as np
from collections import deque
from copy import deepcopy as copy

import torch
import torch.nn as nn
import joblib

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError as e:
    raise ImportError("Failed to import rclpy.") from e

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Int32

from openteach.components import Component
from openteach.utils.network import ZMQKeypointSubscriber


# ─── Model definition (must match training) ──────────────────────────
class SimpleLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes, num_layers):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True, num_layers=num_layers)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        _, (h_n, _) = self.lstm(x)
        return self.fc(h_n[-1])


# ─── Feature computation (matching feature_generate.py exactly) ──────
def calc_trans_velocity(pos_prev, pos_curr):
    """Translational velocity = norm(pos_curr - pos_prev).
    Matches feature_generate.py calc_trans_velocity:
        deltas = pos_seq[1:] - pos_seq[:-1]
        return np.linalg.norm(deltas, axis=1, keepdims=True)
    """
    return np.linalg.norm(pos_curr - pos_prev)


def calc_rot_6d_velocity(rot_prev, rot_curr):
    """6D rotation velocity between two rotation matrices.
    Matches feature_generate.py calc_rot_6d_velocity:
        rot_6d = rot_seq[:, :, :2].transpose(0, 2, 1).reshape(-1, 6)
        rot_6d_diff = rot_6d[1:] - rot_6d[:-1]
    For single frame: first 2 cols -> (3,2), transpose -> (2,3), flatten -> (6,)
    """
    r6d_prev = rot_prev[:, :2].T.reshape(6)
    r6d_curr = rot_curr[:, :2].T.reshape(6)
    return r6d_curr - r6d_prev


def calc_relative_features_rate(pos_L_prev, pos_R_prev, rot_L_prev, rot_R_prev,
                                pos_L_curr, pos_R_curr, rot_L_curr, rot_R_curr):
    """Relative position rate (3,) and relative rotation rate (6,).
    Matches feature_generate.py calc_relative_features_rate:
        pos_local = einsum('nij,ni->nj', rot_L.T, pos_R - pos_L)  =>  rot_L @ (pos_R - pos_L)
        pos_rel_rate = pos_local[1:] - pos_local[:-1]
        rot_rel = einsum('nij,njk->nik', rot_L.T, rot_R)          =>  rot_L.T @ rot_R
        rot_6d = rot_rel[:, :, :2].reshape(N, 6)   (direct reshape, NO transpose)
        rot_rel_rate = rot_6d[1:] - rot_6d[:-1]
    """
    # Relative position in left hand's local frame
    pos_local_prev = rot_L_prev @ (pos_R_prev - pos_L_prev)
    pos_local_curr = rot_L_curr @ (pos_R_curr - pos_L_curr)
    pos_rel_rate = pos_local_curr - pos_local_prev  # (3,)

    # Relative rotation R_L^T @ R_R
    rot_rel_prev = rot_L_prev.T @ rot_R_prev
    rot_rel_curr = rot_L_curr.T @ rot_R_curr

    # 6D: first 2 columns, direct reshape (matches feature_generate.py, NO transpose)
    rot_6d_prev = rot_rel_prev[:, :2].reshape(6)
    rot_6d_curr = rot_rel_curr[:, :2].reshape(6)
    rot_rel_rate = rot_6d_curr - rot_6d_prev  # (6,)

    return pos_rel_rate, rot_rel_rate


# ─── ROS2 Node: subscribe to same topics as DexArmControl ────────────
class CoordinationNode(Node):
    """Single ROS2 node subscribing to both hands' topics.

    Subscribes to the SAME topics that DexArmControl (tesollo_control2.py)
    subscribes to, then assembles state dicts using the SAME logic as
    TesolloRightHand/TesolloLeftHand.get_full_controller_state().
    """

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__('coordination_predictor')

        # QoS matching tesollo_control2.py
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # VR streams are high-frequency; drop-old with best-effort to avoid backlog.
        vr_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Right hand subscriptions ---
        self._right_controller_state = None
        self.create_subscription(
            JointTrajectoryControllerState,
            '/right/dg3f_b_controller/controller_state',
            self._cb_right_ctrl, state_qos)

        self._right_vr_cmd = None
        self.create_subscription(
            JointState, '/right/vr_sent_command',
            self._cb_right_vr_cmd, vr_qos)

        # --- Left hand subscriptions ---
        self._left_controller_state = None
        self.create_subscription(
            JointTrajectoryControllerState,
            '/left/dg3f_b_controller/controller_state',
            self._cb_left_ctrl, state_qos)

        self._left_vr_cmd = None
        self.create_subscription(
            JointState, '/left/vr_sent_command',
            self._cb_left_vr_cmd, vr_qos)

        # Wrist poses are received directly via ZMQ in the main loop,
        # bypassing ROS2 for lower latency.
        self._right_wrist_pose = None
        self._left_wrist_pose = None

        # --- Publisher (BEST_EFFORT to avoid RELIABLE ack overhead) ---
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.coord_pub = self.create_publisher(Int32, '/coordination_mode', pub_qos)

        # Wrist pose cache (same as tesollo_{right,left}.py _last_wrist_pose)
        self._right_last_wrist_pose = None
        self._left_last_wrist_pose = None

        self.get_logger().info("CoordinationNode ready.")

    # ---- Callbacks ----
    def _cb_right_ctrl(self, msg):
        self._right_controller_state = msg

    def _cb_right_vr_cmd(self, msg):
        self._right_vr_cmd = np.array(msg.position, dtype=np.float32)

    def _cb_left_ctrl(self, msg):
        self._left_controller_state = msg

    def _cb_left_vr_cmd(self, msg):
        self._left_vr_cmd = np.array(msg.position, dtype=np.float32)

    def set_wrist_pose(self, hand, wrist_pose):
        """Set wrist pose received from ZMQ (called from main loop)."""
        if hand == 'right':
            self._right_wrist_pose = wrist_pose
        else:
            self._left_wrist_pose = wrist_pose

    # ---- State assembly (matching tesollo_{right,left}.py get_full_controller_state) ----
    def _assemble_hand_state(self, controller_state, vr_cmd, wrist_pose, last_wrist_pose):
        """Assemble state dict using the SAME logic as:
        1. tesollo_control2.py DexArmControl.get_full_controller_state()  (lines 279-324)
        2. tesollo_right.py TesolloRightHand.get_full_controller_state()  (lines 60-88)

        Returns: (state_dict, updated_last_wrist_pose)
        """
        # Step 1: DexArmControl.get_full_controller_state() - read controller_state topic
        if controller_state is None:
            return None, last_wrist_pose

        ref = controller_state.reference   # Desired/commanded
        fb = controller_state.feedback     # Actual/measured
        er = controller_state.error

        if hasattr(controller_state, 'header'):
            timestamp = controller_state.header.stamp.sec + \
                        controller_state.header.stamp.nanosec * 1e-9
        else:
            timestamp = time.time()

        state = dict(
            actual_position=np.array(fb.positions, dtype=np.float32),
            commanded_position=np.array(ref.positions, dtype=np.float32),
            error_position=np.array(er.positions, dtype=np.float32),
            timestamp=timestamp,
        )

        # Step 2: TesolloHand.get_full_controller_state() - replace with VR command
        if vr_cmd is not None:
            state['commanded_position'] = copy(vr_cmd)
            state['error_position'] = vr_cmd - state['actual_position']

        # Step 3: TesolloHand.get_full_controller_state() - add wrist pose with caching
        if wrist_pose is not None:
            last_wrist_pose = copy(wrist_pose)
        if last_wrist_pose is not None:
            state['wrist_position'] = last_wrist_pose[0]
            state['wrist_orientation'] = last_wrist_pose[1:]
        else:
            state['wrist_position'] = np.zeros(3, dtype=np.float32)
            state['wrist_orientation'] = np.zeros((3, 3), dtype=np.float32)

        return state, last_wrist_pose

    def get_right_state(self):
        """Get right hand state (same as TesolloRightHand.get_full_controller_state)."""
        state, self._right_last_wrist_pose = self._assemble_hand_state(
            self._right_controller_state,
            self._right_vr_cmd,
            self._right_wrist_pose,
            self._right_last_wrist_pose,
        )
        return state

    def get_left_state(self):
        """Get left hand state (same as TesolloLeftHand.get_full_controller_state)."""
        state, self._left_last_wrist_pose = self._assemble_hand_state(
            self._left_controller_state,
            self._left_vr_cmd,
            self._left_wrist_pose,
            self._left_last_wrist_pose,
        )
        return state

    def publish_mode(self, mode: int):
        msg = Int32()
        msg.data = mode
        self.coord_pub.publish(msg)

    def shutdown(self):
        try:
            self.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


# ─── Main Component ──────────────────────────────────────────────────
class CoordinationPredictor(Component):
    """
    Runs as a standalone process. Creates a single ROS2 node that subscribes
    to the same topics as DexArmControl, assembles state using the same logic
    as TesolloHand.get_full_controller_state(), computes features at 30Hz
    (matching training data after interpolation), and runs LSTM inference.
    """

    def __init__(self, model_path, scaler_path,
                 host='0.0.0.0',
                 right_keypoints_port=8089,
                 left_keypoints_port=8099,
                 window_size=50,
                 smoothing_count=5, data_frequency=60, feature_frequency=30):
        self.notify_component_start('coordination predictor')

        self.window_size = window_size
        self.smoothing_count = smoothing_count
        self.data_frequency = data_frequency
        self.feature_frequency = feature_frequency
        self._host = host
        self._right_keypoints_port = right_keypoints_port
        self._left_keypoints_port = left_keypoints_port

        # Resolve paths relative to project root if needed
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        if not os.path.isabs(model_path):
            model_path = os.path.join(project_root, model_path)
        if not os.path.isabs(scaler_path):
            scaler_path = os.path.join(project_root, scaler_path)

        self.model_path = model_path
        self.scaler_path = scaler_path

    def _load_model_and_scaler(self):
        """Load LSTM model and StandardScaler."""
        # Force CPU: tiny LSTM is faster on CPU (no transfer overhead),
        # and avoids CUDA contention with the hand detector process.
        self._device = torch.device('cpu')
        # Tiny LSTM doesn't benefit from parallelism; default thread pool
        # (~16 threads) causes severe CPU contention with other teleop processes.
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        self._scaler = joblib.load(self.scaler_path)

        self._model = SimpleLSTM(
            input_dim=25, hidden_dim=128, num_classes=7, num_layers=1
        ).to(self._device)
        self._model.load_state_dict(
            torch.load(self.model_path, map_location=self._device))
        self._model.eval()
        print(f"[CoordinationPredictor] Model loaded from {self.model_path}")
        print(f"[CoordinationPredictor] Scaler loaded from {self.scaler_path}")

    def _compute_feature(self, prev_right, prev_left, curr_right, curr_left):
        """Compute a single 25-dim feature vector from two consecutive frames.

        Uses state dicts assembled with the same logic as
        TesolloHand.get_full_controller_state(), matching feature_generate.py:
            X = np.hstack([feat_9, feat_10, feat_3, feat_4, feat_5, feat_6, feat_11, feat_12])

            feat_9:  mean right gripper error  (1)
            feat_10: mean left gripper error   (1)
            feat_3:  right trans velocity      (1)
            feat_4:  right rot velocity 6D     (6)
            feat_5:  left trans velocity       (1)
            feat_6:  left rot velocity 6D      (6)
            feat_11: relative pos rate         (3)
            feat_12: relative rot rate         (6)
            Total: 25
        """
        # Wrist data (from get_full_controller_state → wrist_position/wrist_orientation)
        pos_R = curr_right['wrist_position']
        rot_R = curr_right['wrist_orientation']
        pos_L = curr_left['wrist_position']
        rot_L = curr_left['wrist_orientation']

        pos_R_prev = prev_right['wrist_position']
        rot_R_prev = prev_right['wrist_orientation']
        pos_L_prev = prev_left['wrist_position']
        rot_L_prev = prev_left['wrist_orientation']

        # feat_9: mean right gripper error (clip negatives to 0, then mean)
        feat_9 = float(np.mean(np.maximum(curr_right['error_position'], 0)))

        # feat_10: mean left gripper error
        feat_10 = float(np.mean(np.maximum(curr_left['error_position'], 0)))

        # feat_3: right translational velocity
        feat_3 = calc_trans_velocity(pos_R_prev, pos_R)

        # feat_4: right rotational velocity (6D)
        feat_4 = calc_rot_6d_velocity(rot_R_prev, rot_R)  # (6,)

        # feat_5: left translational velocity
        feat_5 = calc_trans_velocity(pos_L_prev, pos_L)

        # feat_6: left rotational velocity (6D)
        feat_6 = calc_rot_6d_velocity(rot_L_prev, rot_L)  # (6,)

        # feat_11, feat_12: relative features
        feat_11, feat_12 = calc_relative_features_rate(
            pos_L_prev, pos_R_prev, rot_L_prev, rot_R_prev,
            pos_L, pos_R, rot_L, rot_R
        )

        # Stack in the same order as feature_generate.py
        feature = np.concatenate([
            [feat_9], [feat_10],       # 2
            [feat_3], feat_4,          # 1 + 6 = 7
            [feat_5], feat_6,          # 1 + 6 = 7
            feat_11, feat_12           # 3 + 6 = 9
        ])  # Total: 25

        return feature.astype(np.float32)

    def _predict(self, feature_window):
        """Run LSTM inference on a (window_size, 25) feature window.
        Returns (predicted_class, probabilities_array, timing_dict).
        """
        cpu_start = time.process_time()
        wall_start = time.monotonic()

        normalized = self._scaler.transform(feature_window)
        t_scaler = time.monotonic()

        x = torch.FloatTensor(normalized).unsqueeze(0).to(self._device)
        t_tensor = time.monotonic()

        with torch.no_grad():
            output = self._model(x)
            t_model = time.monotonic()
            probs = torch.softmax(output, dim=1).cpu().numpy()[0]
            pred = int(np.argmax(probs))

        cpu_end = time.process_time()
        wall_end = time.monotonic()

        timing = {
            'scaler': (t_scaler - wall_start) * 1000,
            'tensor': (t_tensor - t_scaler) * 1000,
            'model': (t_model - t_tensor) * 1000,
            'post': (wall_end - t_model) * 1000,
            'cpu_ms': (cpu_end - cpu_start) * 1000,
            'wall_ms': (wall_end - wall_start) * 1000,
        }
        return pred, probs, timing

    def stream(self):
        """Main loop: subscribe to topics at 30Hz, compute features, infer.

        Training pipeline:
            DexArmControl subscribes to topics → get_full_controller_state() @ 60Hz
            → save H5 → align_data.py interpolates to 30fps → feature_generate.py
        Inference pipeline (this):
            CoordinationNode subscribes to SAME topics → assemble state (same logic)
            → sample @ 30Hz (matching aligned training data) → same features → LSTM
        """
        # Create single ROS2 node (controller_state + vr_cmd via ROS2)
        print("[CoordinationPredictor] Creating ROS2 node...")
        self._node = CoordinationNode()

        # ZMQ subscribers for wrist pose (direct from transform process, bypass ROS2)
        self._right_frame_sub = ZMQKeypointSubscriber(
            host=self._host,
            port=self._right_keypoints_port,
            topic='transformed_hand_frame',
        )
        self._left_frame_sub = ZMQKeypointSubscriber(
            host=self._host,
            port=self._left_keypoints_port,
            topic='transformed_hand_frame',
        )
        print(f"[CoordinationPredictor] ZMQ wrist pose: "
              f"right={self._host}:{self._right_keypoints_port}, "
              f"left={self._host}:{self._left_keypoints_port}")

        # Load model and scaler
        self._load_model_and_scaler()

        # Loop timing: 30Hz using time.sleep() instead of busy-wait
        # to avoid holding the GIL and starving ROS2 callback processing.
        loop_period = 1.0 / self.feature_frequency

        feature_buffer = deque(maxlen=self.window_size)
        prev_right = None
        prev_left = None

        # Smoothing state
        current_mode = 0
        candidate_mode = 0
        candidate_count = 0

        LABEL_NAMES = ['NoAct', 'Loose', 'UniL', 'UniR', 'TAsyL', 'TAsyR', 'TSym']

        window_duration = self.window_size / self.feature_frequency
        print(f"[CoordinationPredictor] Feature rate: {self.feature_frequency} Hz")
        print(f"[CoordinationPredictor] Window: {self.window_size} frames = "
              f"{window_duration:.2f}s, Smoothing: {self.smoothing_count}")

        frame_idx = 0
        waiting_printed = False

        while True:
            try:
                loop_start = time.monotonic()

                # Drain ALL pending ROS2 callbacks in the main thread.
                ts0 = time.monotonic()
                for _ in range(12):
                    rclpy.spin_once(self._node, timeout_sec=0)
                ts1 = time.monotonic()

                # Receive wrist poses directly from ZMQ (non-blocking)
                right_frame = self._right_frame_sub.recv_keypoints(flags=1)
                if right_frame is not None:
                    right_frame = np.asanyarray(right_frame).reshape(4, 3)
                    self._node.set_wrist_pose('right', right_frame)

                left_frame = self._left_frame_sub.recv_keypoints(flags=1)
                if left_frame is not None:
                    left_frame = np.asanyarray(left_frame).reshape(4, 3)
                    self._node.set_wrist_pose('left', left_frame)
                ts2 = time.monotonic()

                # Assemble state using same logic as get_full_controller_state()
                right_state = self._node.get_right_state()
                left_state = self._node.get_left_state()

                if right_state is None or left_state is None:
                    if not waiting_printed:
                        missing = []
                        if right_state is None:
                            missing.append('right')
                        if left_state is None:
                            missing.append('left')
                        print(f"[CoordinationPredictor] Waiting for data: {missing} not ready")
                        waiting_printed = True
                    self._node.publish_mode(current_mode)
                    # Sleep until next loop iteration
                    elapsed = time.monotonic() - loop_start
                    sleep_time = loop_period - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                if waiting_printed:
                    print("[CoordinationPredictor] All data ready, starting collection.")
                    waiting_printed = False

                # Compute feature between consecutive 30Hz frames
                ts3 = time.monotonic()
                if prev_right is not None and prev_left is not None:
                    feature = self._compute_feature(
                        prev_right, prev_left, right_state, left_state)
                    feature_buffer.append(feature)
                ts4 = time.monotonic()

                prev_right = right_state
                prev_left = left_state

                # Need full window to predict
                if len(feature_buffer) < self.window_size:
                    if len(feature_buffer) % 10 == 0:
                        print(f"[CoordinationPredictor] Filling buffer: "
                              f"{len(feature_buffer)}/{self.window_size}")
                    self._node.publish_mode(current_mode)
                    elapsed = time.monotonic() - loop_start
                    sleep_time = loop_period - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                frame_idx += 1

                # --- TIMING DIAGNOSTICS ---
                t0 = time.monotonic()

                # Run inference
                feature_window = np.array(feature_buffer)  # (50, 25)
                t1 = time.monotonic()

                raw_pred, probs, infer_timing = self._predict(feature_window)
                t2 = time.monotonic()

                # Apply smoothing
                if raw_pred == candidate_mode:
                    candidate_count += 1
                else:
                    candidate_mode = raw_pred
                    candidate_count = 1

                if candidate_count >= self.smoothing_count:
                    current_mode = candidate_mode

                self._node.publish_mode(current_mode)
                t3 = time.monotonic()

                # Print every 30 frames (1 sec)
                if frame_idx % 30 == 0:
                    dt_total = (t3 - loop_start) * 1000
                    it = infer_timing
                    print(f"[F{frame_idx:04d}] "
                          f"scaler={it['scaler']:.1f} tensor={it['tensor']:.1f} "
                          f"model={it['model']:.1f} post={it['post']:.1f} | "
                          f"CPU={it['cpu_ms']:.1f}ms WALL={it['wall_ms']:.1f}ms | "
                          f"TOTAL={dt_total:.1f}ms | "
                          f"smooth={LABEL_NAMES[current_mode]}")

                # Sleep-based timing (releases CPU and GIL)
                elapsed = time.monotonic() - loop_start
                sleep_time = loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except KeyboardInterrupt:
                break

        print("[CoordinationPredictor] Shutting down.")
        self._right_frame_sub.stop()
        self._left_frame_sub.stop()
        self._node.shutdown()
