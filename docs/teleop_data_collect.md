# Running the teleop

#### For Robots


To use the OpenTeach teleop module with robots, open the VR Application in your Oculus Headset. On the robot server side, start the controllers first for robots followed by the following command to start the teleop :

Franka and Kinova mirrors human arm motions and retargets it to the robot.

##### Allegro Hand 

`python teleop.py robot=allegro`

##### Bimanual XARM 

`python teleop.py robot=bimanual`

##### Franka

`python teleop.py robot=franka`

##### Kinova

`python teleop.py robot=kinova`

##### Xela Hand

`python teleop.py robot=xela_hand run_xela=True`

##### Allegro Franka

`python teleop.py robot=allegro_franka`

##### Xela Hand Franka

`python teleop.py robot=xela_hand_franka run_xela=True`

##### Allegro Kinova

`python teleop.py robot=allegro_kinova`




#### For Simulation 

To use the Open-Teach teleop module, open the VR Application in your Oculus Headset. On the server side, use the following command to start the teleop :

##### Allegro Sim

`python teleop.py robot=allegro_sim sim_env=True`

##### Libero Sim

`python teleop.py robot=libero_sim sim_env=True`

# Data Collection

The Data Collection module saves the robot states , cameras sensors output as a video and sensor states.

#### For Robots 

##### Without Xela Sensors

`python data_collect.py robot=allegro/kinova/franka/allegro_franka/allegro_kinova/bimanual demo_num=1`

##### With Xela Sensors

`python data_collect.py robot=xela_hand_franka demo_num=1 is_xela=True`

#### For Simulation

`python3 data_collect.py robot=allegro_sim/libero_sim demo_num=1 sim_env=True`

#### Note: Remember to enter your network IP on the server [config](/configs/network.yaml)

The data saves camera stream in the optimized form of .avi videos and saves depth and robot information in the form of h5 files.

# Synchronizing Demonstrations

Each recorder runs at a different rate, which means raw files inside `extracted_data/demonstration_<N>` do not share the same timestamps. Use `sync_demonstration.py` to resample the robot states to the RGB frame timestamps and produce a dataset where every RGB frame has matching joint data:

```bash
python sync_demonstration.py --demo 11 --storage-path extracted_data
```

The command above creates `extracted_data/demonstration_11/synced_cam_0/` containing:
- `synced_data.h5` with the RGB timestamps, optional frame file paths, and a resampled copy of every `<robot>_<state>.h5` dataset.
- `synced_index.csv` to quickly inspect `frame_idx`, `timestamp`, and `relative_time` during labeling.

If you need individual RGB images when annotating, add `--dump-frames` and the script will decode `cam_<idx>_rgb_video.avi` into `rgb_frames/frame_*.jpg` (set `--image-ext` if you prefer png).
