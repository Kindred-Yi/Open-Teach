import os
import time
import h5py
import numpy as np
from .recorder import Recorder
from openteach.utils.timer import FrequencyTimer
from openteach.constants import VR_FREQ
from openteach.utils.network import ZMQKeypointSubscriber


class VRPositionRecorder(Recorder):
    """
    Recorder for VR hand positions and their relative displacement.

    Records:
    - Right hand wrist position (VR space)
    - Left hand wrist position (VR space)
    - Position difference (left - right)
    - Timestamps

    Data is naturally aligned since both hands are read in the same loop iteration.
    """

    def __init__(
        self,
        host,
        right_transform_port,
        left_transform_port,
        storage_path,
        frequency=VR_FREQ,
    ):
        self._filename = 'vr_position_diff'
        self.notify_component_start('{}'.format(self._filename))
        self._recorder_file_name = os.path.join(storage_path, self._filename + '.h5')

        # Subscriber for right hand transformed frame
        self.right_frame_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=right_transform_port,
            topic='transformed_hand_frame'
        )

        # Subscriber for left hand transformed frame
        self.left_frame_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=left_transform_port,
            topic='transformed_hand_frame'
        )

        self.timer = FrequencyTimer(frequency)

        # Data containers
        self.data = {
            'right_wrist_position': [],
            'left_wrist_position': [],
            'position_diff': [],  # left - right
            'timestamps': [],
        }

    def _extract_wrist_position(self, frame_data):
        """
        Extract wrist position from transformed_hand_frame.
        Frame format: [origin_coord, cross_product, palm_normal, palm_direction]
        origin_coord (first row) is the wrist position in VR space.
        """
        frame = np.asanyarray(frame_data).reshape(4, 3)
        return frame[0]  # origin_coord = wrist position

    def stream(self):
        print('Checking if VR transform ports are active...')

        # Wait for both ports to be active
        while True:
            right_data = self.right_frame_subscriber.recv_keypoints()
            left_data = self.left_frame_subscriber.recv_keypoints()
            if right_data is not None and left_data is not None:
                break
            time.sleep(0.01)

        print('Starting to record VR positions to store in {}.'.format(self._recorder_file_name))

        self.num_datapoints = 0
        self.record_start_time = time.time()

        while True:
            self.timer.start_loop()
            try:
                # Receive transformed frames from both hands
                right_frame = self.right_frame_subscriber.recv_keypoints()
                left_frame = self.left_frame_subscriber.recv_keypoints()
                timestamp = time.time()

                if right_frame is not None and left_frame is not None:
                    # Extract wrist positions
                    right_wrist = self._extract_wrist_position(right_frame)
                    left_wrist = self._extract_wrist_position(left_frame)

                    # Calculate position difference (left - right)
                    pos_diff = left_wrist - right_wrist

                    # Store data
                    self.data['right_wrist_position'].append(right_wrist.astype(np.float32))
                    self.data['left_wrist_position'].append(left_wrist.astype(np.float32))
                    self.data['position_diff'].append(pos_diff.astype(np.float32))
                    self.data['timestamps'].append(timestamp)

                    self.num_datapoints += 1

                self.timer.end_loop()
            except KeyboardInterrupt:
                self.record_end_time = time.time()
                break

        # Display statistics
        self._display_statistics(self.num_datapoints)

        # Save metadata
        self._add_metadata(self.num_datapoints)

        # Write to dataset
        print('Compressing VR position data...')
        with h5py.File(self._recorder_file_name, "w") as file:
            for key in self.data.keys():
                if key != 'timestamps':
                    arr = np.array(self.data[key], dtype=np.float32)
                    file.create_dataset(key + 's', data=arr, compression="gzip", compression_opts=6)
                else:
                    arr = np.array(self.data[key], dtype=np.float64)
                    file.create_dataset('timestamps', data=arr, compression="gzip", compression_opts=6)

            # Other metadata
            file.update(self.metadata)

        print('Saved VR position data in {}.'.format(self._recorder_file_name))
