from franka_arm.controller import FrankaController
from copy import deepcopy as copy

def main():
    franka = FrankaController(hand_type="right")
    joint_positions = copy(franka.get_joint_position())
    print('joint_position: {}'.format(joint_positions))

if __name__ == "__main__":
    main()
