import h5py

with h5py.File('aligned_data.h5', 'r') as f:
    print('Labels length:', len(f['coordination_labels']))
    print('Right arm positions:', f['right_arm_cart']['positions'].shape)
    print('Left arm positions:', f['left_arm_cart']['positions'].shape)
    print('Right arm orientations:', f['right_arm_cart']['orientations'].shape)
    print('Left arm orientations:', f['left_arm_cart']['orientations'].shape)
    print('Right gripper:', f['right_gripper']['positions'].shape)
    print('Left gripper:', f['left_gripper']['positions'].shape)
