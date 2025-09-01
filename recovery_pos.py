import sys
from franka_arm.controller import FrankaController
from copy import deepcopy as copy

RECOVERY_POSITION = [ 6.07320042e-04,2.71967704e-04, -8.25914538e-05, -1.57061762e+00,
  2.06199921e-04,  1.56913572e+00,  7.85077823e-01]

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("left", "right"):
        print("Usage: python reset_pos.py [left|right]")
        sys.exit(1)

    hand_type = sys.argv[1]
    franka = FrankaController(hand_type=hand_type)
    franka.joint_movement(RECOVERY_POSITION)

if __name__ == "__main__":
    main()
