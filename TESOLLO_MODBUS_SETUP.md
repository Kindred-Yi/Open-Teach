# Tesollo 3F Gripper Modbus TCP Setup Guide

## Overview
This guide explains how to use the Tesollo 3F gripper with direct Modbus TCP communication instead of ROS.

## Prerequisites

### Hardware Requirements
- Tesollo DG-3F gripper
- Ethernet connection to the gripper
- Default gripper IP: `169.254.186.72`
- Default port: `502`

### Software Dependencies
```bash
pip install pymodbus
```

## Quick Start

### 1. Test Connection (Dummy Mode)
```python
python test_tesollo_modbus.py
```

### 2. Test with Real Hardware
```python
from openteach.robot.tesollo.tesollo import TesolloHand

# Create hand controller
hand = TesolloHand(ip='169.254.186.72', port=502, dummy=False)

# Basic operations
hand.home()  # Move to home position
hand.set_random_position()  # Move to random position

# Custom movement (12 joints: 3 fingers × 4 joints each)
angles = [0.1, 0.2, 0.3, 0.1] * 3  # Small angles for each finger
hand.move(angles)

# Get current position
position = hand.get_joint_position()
print(f"Current position: {position}")
```

## Configuration Files

### Created Configuration Files:
- `openteach/robot/tesollo/configs/tesollo_info.yaml` - Basic gripper info
- `openteach/robot/tesollo/configs/tesollo_bounds.yaml` - Movement bounds and scaling
- `openteach/robot/tesollo/configs/tesollo_link_info.yaml` - Joint limits and offsets

### Key Parameters in tesollo_bounds.yaml:
```yaml
# Safe movement bounds (max change per step)
jointwise_angle_bounds: 
  - 0.05  # Rotatory joints (conservative)
  - 0.2   # Other joints

# Human-to-robot mapping scales
linear_scaling_factors: [1.0, 1.0, 1.0]
rotatory_scaling_factors: 
  index: 0.02   # F1 
  middle: 0.02  # F2
  thumb: 0.05   # F3 (more sensitive)
```

## Modified Files

### Core Controller:
- `openteach/ros_links/tesollo_control.py` - New Modbus TCP controller
- `openteach/robot/tesollo/tesollo.py` - Updated to use Modbus controller
- `openteach/robot/tesollo/tesollo_retargeters.py` - Tesollo-specific retargeters

### Operator:
- `openteach/components/operators/tesollo.py` - Updated to use TesolloHand

## Key Features

### Direct Communication
- **No ROS dependency** for gripper control
- **Real-time Modbus TCP** communication
- **Background state updates** at 100Hz

### Safety Features
- **Bounded movements** prevent joint damage
- **Angle conversion** (degrees ↔ radians)
- **Error handling** for connection issues
- **Dummy mode** for testing without hardware

### Compatibility
- **Same interface** as original Allegro controller
- **Drop-in replacement** for existing code
- **Supports teleoperation** with human hand mapping

## Network Configuration

### Default Settings:
- **IP Address**: `169.254.186.72`
- **Port**: `502`
- **Slave ID**: `1`

### Changing IP Address:
```python
from openteach.ros_links.tesollo_control import TesolloModbusCommunication

comm = TesolloModbusCommunication()
comm.connect('169.254.186.72', 502)
comm.set_ip('169.254.186.73')  # New IP
comm.rom_write()  # Save to EEPROM

# Restart gripper power to apply new IP
```

## Troubleshooting

### Connection Issues:
1. **Check network connection**
   - Ping the gripper: `ping 169.254.186.72`
   - Ensure correct subnet (usually 169.254.x.x)

2. **Check firewall**
   - Allow port 502 (Modbus TCP)
   - Disable firewall temporarily for testing

3. **Test with dummy mode**
   - Use `dummy=True` parameter for testing

### Performance Issues:
1. **Reduce update frequency** if needed
2. **Check network latency**
3. **Adjust scaling factors** in configuration

### Joint Limit Errors:
1. **Check joint bounds** in `tesollo_link_info.yaml`
2. **Adjust movement bounds** in `tesollo_bounds.yaml`
3. **Use smaller scaling factors**

## Advanced Usage

### Custom IP and Port:
```python
hand = TesolloHand(ip='192.168.1.100', port=502)
```

### Free Mode (Manual Movement):
```python
hand._controller.set_hand_free(True)  # Allow manual movement
hand._controller.set_hand_free(False) # Lock joints
```

### Direct Modbus Access:
```python
# Access low-level Modbus communication
modbus = hand._controller.tesollo
positions = modbus.get_position()  # Get positions in degrees
modbus.set_position([10, 20, 30, 0] * 3)  # Set positions in degrees
```

## Integration with OpenTeach

The Tesollo controller is now fully integrated with the OpenTeach framework:

```python
from openteach.components.operators.tesollo import TesolloHandOperator

# Use in teleoperation
operator = TesolloHandOperator(
    host='localhost',
    transformed_keypoints_port=5555,
    finger_configs={
        'freeze_index': False,
        'freeze_middle': False, 
        'freeze_thumb': False,
        'no_index': False,
        'no_middle': False,
        'no_thumb': False
    }
)
```

## Support

For issues or questions:
1. Check error messages in console
2. Test with dummy mode first
3. Verify network connectivity
4. Check configuration files