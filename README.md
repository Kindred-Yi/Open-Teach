# Original Paper: OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation

[Paper](https://arxiv.org/abs/2403.07870) [Website](https://open-teach.github.io/)

This is a fork of the implementation of the Open Teach including unity scripts for the VR application, teleoperation pipeline and demonstration collection pipeline.

Open Teach consists of two parts. 

- [x] Teleoperation using Meta Quest 3 and data collection over a range of robot morphologies and simulation environments.

- [x] Policy training for various dexterous manipulation tasks across different robots and simulations.

### VR Code and User Interface

Read VR specific information, User Interface and APK files [here](/docs/vr.md)

### Server Code Installation 

Install the conda environment from the yaml file in the codebase

`conda env create -f environment.yml`

This will install all the dependencies required for the server code.  

After installing all the prerequisites, you can install this pipeline as a package with pip:

`pip install -e . `

You can test if it had installed correctly by running ` import openteach` from the python shell.

### Robot Controller Installation Specific Information

1. For Simulation specific information, follow the instructions [here](/docs/simulation.md).

2. For Robot controller installation, follow the instructions [here](https://github.com/NYU-robot-learning/OpenTeach-Controllers)

### For starting the camera sensors

For starting the camera sensors and streaming them inside the screen in the oculus refer [here](/docs/sensors.md)

### Running the Teleoperation and Data Collection

For information on running the teleoperation and data collection refer [here](/docs/teleop_data_collect.md).


### Policy Learning 

For open-source code of the policies we trained on the robots refer [here](/docs/policy_learning.md) 

### Policy Learning API

For using the API we use for policy learning, use [this](https://github.com/NYU-robot-learning/Open-Teach-API)


## Running Teleoperation(Kindred's Note)
IP configurationn:
Right Franka Arm: 192.168.4.2

Left Franka Arm: 192.168.4.3

NUC(the laptop with real-time kernel): 192.168.4.4

PC(your own laptop): 192.168.4.6

First, you need to connect to the NUC:
```bash
ssh hcilab@192.168.4.4
```

```bash
cd Desktop/github/deoxys_control/deoxys
./bin/franka-interface config/charmander_left.yml # you need to open a new terminal for starting the right arm controller node
```
Then in your PC:
First install miniconda [here](https://www.anaconda.com/docs/getting-started/miniconda/install#macos-linux-installation)

In this repo, activate the conda environment:
```bash
conda env create -f environment.yml
conda activate openteach
```
This will install all the dependencies required for the server code.  
After installing all the prerequisites, you can install this pipeline as a package with pip:
`pip install -e . `

Next, install deoxys for franka control in your PC by following the (documentation)[https://zhuyifengzju.github.io/deoxys_docs/html/installation/codebase_installation.html], it's already installed in NUC

Then we can start teloperating by running:
```bash
cd Open-Teach/
python teleop.py robot=dual_franka
```
When you can see everything is working in terminal, you can start the VR program in your headset
