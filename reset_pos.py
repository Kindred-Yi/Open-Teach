import sys
from franka_arm.controller import FrankaController
from copy import deepcopy as copy

RESET_POSITION = [
    0.09162008114028396,
    -0.19826458111314524,
    -0.01990020486871322,
    -2.4732269941140346,
    -0.01307073642274261,
    2.30396583422025,
    0.8480939705504309,
]

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("left", "right"):
        print("Usage: python reset_pos.py [left|right]")
        sys.exit(1)

    hand_type = sys.argv[1]
    franka = FrankaController(hand_type=hand_type)
    franka.joint_movement(RESET_POSITION)

if __name__ == "__main__":
    main()
