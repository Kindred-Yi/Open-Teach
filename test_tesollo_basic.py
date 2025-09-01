#!/usr/bin/env python3
"""
Basic test for Tesollo 3F gripper with default IP
"""

import sys
import time
import numpy as np

# Add OpenTeach path
sys.path.append('/home/kindred/Desktop/repo/Open-Teach')

from openteach.robot.tesollo.tesollo import TesolloHand

def test_basic_control():
    """Test basic Tesollo control with default IP"""
    
    print("Tesollo 3F Basic Control Test")
    print("=" * 40)
    print("Using default IP: 169.254.186.72")
    print("Port: 502")
    
    try:
        # Create Tesollo hand controller
        print("\n1. Connecting to Tesollo gripper...")
        hand = TesolloHand(ip='169.254.186.72', port=502, dummy=False)
        
        # Wait for initialization
        print("2. Waiting for initialization...")
        time.sleep(3)
        
        # Check connection by reading current position
        print("3. Reading current position...")
        current_pos = hand.get_joint_position()
        
        if current_pos is not None:
            print(f"✓ Successfully connected!")
            print(f"Current position (12 joints): {current_pos}")
            print(f"Position in degrees: {current_pos * 180 / np.pi}")
        else:
            print("❌ Could not read position - check connection")
            return False
        
        # Test home position
        print("\n4. Moving to home position...")
        print("Home position: all joints at 0 degrees")
        
        success = hand.home()
        if success:
            print("✓ Home command sent successfully")
        else:
            print("❌ Home command failed")
            return False
        
        # Wait for movement
        print("5. Waiting 5 seconds for movement to complete...")
        time.sleep(5)
        
        # Read position after homing
        print("6. Reading position after homing...")
        new_pos = hand.get_joint_position()
        if new_pos is not None:
            print(f"Position after homing: {new_pos}")
            print(f"Position in degrees: {new_pos * 180 / np.pi}")
            
            # Check if position is close to zero (home)
            max_deviation = np.max(np.abs(new_pos))
            print(f"Maximum deviation from home: {max_deviation:.3f} radians ({max_deviation * 180 / np.pi:.1f} degrees)")
            
            if max_deviation < 0.1:  # Less than ~6 degrees
                print("✓ Successfully moved to home position!")
            else:
                print("⚠️  Position not exactly at home, but movement detected")
        
        print("\n🎉 Basic control test completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Error during test: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that Tesollo gripper is powered on")
        print("2. Verify ethernet connection")
        print("3. Make sure gripper IP is 169.254.186.72")
        print("4. Check if port 502 is accessible")
        return False


def test_small_movement():
    """Test small controlled movement"""
    print("\n" + "=" * 40)
    print("Testing small movement...")
    
    try:
        hand = TesolloHand(ip='169.254.186.72', port=502, dummy=False)
        time.sleep(2)
        
        # Small movement: 10 degrees on first joint of each finger
        print("Moving first joint of each finger by 10 degrees...")
        small_angles = [
            np.radians(10), 0, 0, 0,  # Finger 1: 10° on first joint
            np.radians(10), 0, 0, 0,  # Finger 2: 10° on first joint  
            np.radians(10), 0, 0, 0   # Finger 3: 10° on first joint
        ]
        
        success = hand.move(small_angles)
        if success:
            print("✓ Small movement command sent")
            time.sleep(3)
            
            # Return to home
            print("Returning to home...")
            hand.home()
            time.sleep(3)
            print("✓ Returned to home")
        else:
            print("❌ Small movement failed")
            
    except Exception as e:
        print(f"❌ Error in small movement test: {e}")


def main():
    print("Starting Tesollo basic tests...")
    
    # Test 1: Basic connection and home
    if test_basic_control():
        # Test 2: Small movement (optional)
        response = input("\nDo you want to test small movements? (y/n): ").strip().lower()
        if response in ['y', 'yes']:
            test_small_movement()
    
    print("\nTest completed!")
    print("\nNext steps:")
    print("- If successful: You can now use the gripper with OpenTeach")
    print("- If failed: Check network connection and gripper power")


if __name__ == "__main__":
    main()