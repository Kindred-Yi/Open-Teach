import numpy as np
import time
import sys
import os
import threading
import struct
from copy import deepcopy as copy

# Add the path to access delto_modbus_TCP
sys.path.append('/home/kindred/Desktop/repo/OpenTeach-Controllers/src/tesollo_controllers/delto_3f_driver/delto_utility')

try:
    from delto_3f_enum import Delto3F, Delto3FCoils, Delto3FHoldingRegisters, Delto3FInputRegisters
    from pymodbus.client.tcp import ModbusTcpClient
except ImportError as e:
    print(f"Warning: Could not import Modbus dependencies: {e}")
    print("Running in dummy mode")


# Tesollo 3F home position values (12 joints: 3 fingers × 4 joints each)
TESOLLO_ORIGINAL_HOME_VALUES = [
    0, 0, 0, 0,        # Finger 1 (Index equivalent)
    0, 0, 0, 0,        # Finger 2 (Middle equivalent)  
    0, 0, 0, 0         # Finger 3 (Thumb equivalent)
]

# Default network configuration
DEFAULT_TESOLLO_IP = '169.254.186.72'
DEFAULT_TESOLLO_PORT = 502


class TesolloModbusCommunication:
    '''
    Direct Modbus TCP communication with Tesollo 3F gripper
    Based on Communication class from delto_modbus_TCP.py
    '''

    def __init__(self, dummy=False):
        self.client = None
        self.slaveID = 0
        self.dummy = dummy
        self.lock = threading.Lock()
        
        # Cache for current state
        self._current_position = None
        self._current_velocity = None
        self._current_effort = None
        self._last_commanded_position = None
        self._last_update_time = None

    def __del__(self):
        self.disconnect()

    def connect(self, ip=DEFAULT_TESOLLO_IP, port=DEFAULT_TESOLLO_PORT, slaveID=1):
        '''Connect to Tesollo Gripper'''
        self.slaveID = slaveID
        if self.dummy:
            print(f"[DUMMY] Connecting to Tesollo at {ip}:{port}")
            return True

        try:
            self.client = ModbusTcpClient(host=ip, port=port)
            result = self.client.connect()
            if result:
                print(f"Successfully connected to Tesollo at {ip}:{port}")
            else:
                print(f"Failed to connect to Tesollo at {ip}:{port}")
            return result
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        '''Disconnect from Tesollo Gripper'''
        if self.dummy:
            print("[DUMMY] Disconnecting from Tesollo")
            return

        if self.client:
            try:
                self.client.close()
            except:
                pass

    def get_position(self):
        '''Get current joint positions in degrees'''
        if self.dummy:
            # Return dummy positions for 12 joints
            return [0.0] * 12

        try:
            with self.lock:
                status = self.client.read_input_registers(
                    address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value,
                    count=Delto3F.MOTOR_NUM.value,  # 12 motors
                    slave=self.slaveID).registers

                # Convert from unsigned 16-bit to signed and scale
                for i in range(Delto3F.MOTOR_NUM.value):
                    status[i] = (status[i] if status[i] < 32768 else status[i] - 65536) / 10.0
                
                self._current_position = status
                self._last_update_time = time.time()
                return status
        except Exception as e:
            print(f"Error reading position: {e}")
            return None

    def set_position(self, position):
        '''Set target joint positions in degrees'''
        if len(position) != 12:
            print("Error: Position array must have 12 elements")
            return False

        if self.dummy:
            print(f"[DUMMY] Setting position: {position}")
            self._last_commanded_position = position
            return True

        try:
            with self.lock:
                # Convert float degrees to integer (0.1 degree precision)
                intPosition = list(map(lambda x: struct.unpack(
                    'H', struct.pack('h', int((x * 10))))[0], position))
                
                self.client.write_registers(
                    address=72, values=intPosition, slave=self.slaveID)
                
                self._last_commanded_position = position
                return True
        except Exception as e:
            print(f"Error setting position: {e}")
            return False

    def set_free_mode(self, is_free):
        '''Set joints to free/locked mode'''
        if self.dummy:
            print(f"[DUMMY] Setting free mode: {is_free}")
            return

        try:
            self.client.write_coil(
                address=Delto3FCoils.JOINT_CUSTOM_MODE.value, 
                value=is_free, 
                slave=self.slaveID)
        except Exception as e:
            print(f"Error setting free mode: {e}")


class DexArmControl():
    def __init__(self, record_type=None, robot_type='both', ip=None, port=None, dummy=False):
        '''
        Initialize Tesollo control using direct Modbus TCP communication
        
        Args:
            robot_type: Should be 'tesollo' for Tesollo gripper
            ip: Tesollo gripper IP address (default: 169.254.186.72)
            port: Tesollo gripper port (default: 502)
            dummy: Run in dummy mode for testing
        '''
        self.robot_type = robot_type
        
        if robot_type == 'tesollo':
            self._init_tesollo_hand_control(ip, port, dummy)

    def _init_tesollo_hand_control(self, ip=None, port=None, dummy=False):
        '''Initialize Tesollo hand control with Modbus TCP'''
        ip = ip or DEFAULT_TESOLLO_IP
        port = port or DEFAULT_TESOLLO_PORT
        
        print(f"Initializing Tesollo hand control via Modbus TCP at {ip}:{port}")
        
        self.tesollo = TesolloModbusCommunication(dummy=dummy)
        
        # Try to connect
        if not self.tesollo.connect(ip, port):
            print("Warning: Could not connect to Tesollo gripper")
        
        # Initialize state variables
        self.tesollo_joint_state = None
        self.tesollo_commanded_joint_state = None
        
        # Start background thread for state updates
        self._running = True
        self._update_thread = threading.Thread(target=self._update_state_loop, daemon=True)
        self._update_thread.start()

    def _update_state_loop(self):
        '''Background thread to periodically update joint states'''
        while self._running:
            try:
                # Update current position
                positions = self.tesollo.get_position()
                if positions is not None:
                    # Convert degrees to radians for consistency
                    positions_rad = np.array(positions) * np.pi / 180.0
                    
                    # Create joint state dictionary
                    self.tesollo_joint_state = {
                        'position': positions_rad.astype(np.float32),
                        'velocity': np.zeros(12, dtype=np.float32),  # Velocity not available
                        'effort': np.zeros(12, dtype=np.float32),    # Effort not available
                        'timestamp': time.time()
                    }
                    
                    # Update commanded state if available
                    if self.tesollo._last_commanded_position is not None:
                        commanded_rad = np.array(self.tesollo._last_commanded_position) * np.pi / 180.0
                        self.tesollo_commanded_joint_state = {
                            'position': commanded_rad.astype(np.float32),
                            'velocity': np.zeros(12, dtype=np.float32),
                            'effort': np.zeros(12, dtype=np.float32),
                            'timestamp': time.time()
                        }
                
            except Exception as e:
                print(f"Error in state update loop: {e}")
            
            time.sleep(0.01)  # 100Hz update rate

    def stop(self):
        '''Stop the background update thread'''
        self._running = False
        if hasattr(self, '_update_thread'):
            self._update_thread.join(timeout=1.0)

    # State information functions
    def get_hand_state(self):
        '''Get complete hand state'''
        return self.tesollo_joint_state

    def get_commanded_hand_state(self):
        '''Get commanded hand state'''
        return self.tesollo_commanded_joint_state
        
    def get_hand_position(self):
        '''Get current joint positions in radians'''
        if self.tesollo_joint_state is None:
            return None
        return self.tesollo_joint_state['position']

    def get_hand_velocity(self):
        '''Get current joint velocities (not available for Modbus, returns zeros)'''
        if self.tesollo_joint_state is None:
            return None
        return self.tesollo_joint_state['velocity']

    def get_hand_torque(self):
        '''Get current joint torques (not available for Modbus, returns zeros)'''
        if self.tesollo_joint_state is None:
            return None
        return self.tesollo_joint_state['effort']

    def get_commanded_hand_joint_position(self):
        '''Get commanded joint positions'''
        if self.tesollo_commanded_joint_state is None:
            return None
        return self.tesollo_commanded_joint_state['position']

    # Movement functions
    def move_hand(self, tesollo_angles):
        '''
        Move hand to specified joint angles
        
        Args:
            tesollo_angles: Array of 12 joint angles in radians
        '''
        if len(tesollo_angles) != 12:
            print("Error: Tesollo requires exactly 12 joint angles")
            return False
            
        # Convert radians to degrees for Modbus communication
        angles_deg = np.array(tesollo_angles) * 180.0 / np.pi
        return self.tesollo.set_position(angles_deg.tolist())

    def home_hand(self):
        '''Move hand to home position'''
        return self.move_hand(TESOLLO_ORIGINAL_HOME_VALUES)

    def reset_hand(self):
        '''Reset hand to home position'''
        return self.home_hand()

    def set_hand_free(self, is_free=True):
        '''Set hand to free mode (joints can move freely)'''
        self.tesollo.set_free_mode(is_free)
    
    # Full robot commands (for compatibility)
    def move_robot(self, tesollo_angles, arm_angles=None):
        '''Move robot (only hand for Tesollo)'''
        return self.move_hand(tesollo_angles)

    def home_robot(self):
        '''Home robot (only hand for Tesollo)'''
        return self.home_hand()

    def __del__(self):
        '''Cleanup when object is destroyed'''
        try:
            self.stop()
            if hasattr(self, 'tesollo'):
                self.tesollo.disconnect()
        except:
            pass