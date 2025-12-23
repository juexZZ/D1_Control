import numpy as np

# # Target pose in camera coordinate system

T_target_in_cam = np.array([
    [ 0.02277477, -0.99893260, -0.04018656,  0.01049047],
    [ 0.49252543,  0.04619145, -0.86907136, -0.06553812],
    [ 0.87000000,  0.00000000,  0.49305171,  0.23500000],
    [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
])

# Camera pose in robot arm base frame

T_cam_in_base = np.array([
    [ 0.01854991, -0.22026411,  0.97526387,  0.36235041],
    [-0.99972266,  0.01006837,  0.02128908, -0.00776866],
    [-0.01450853, -0.97538831, -0.22001626,  0.02456633],
    [ 0.        ,  0.        ,  0.        ,  1.        ],
])

# Axis alignment transformation
T_align = np.array([
    [0,  0,  1,  0],
    [0,  -1,  0,  0],
    [1,  0,  0,  0],
    [0,  0,  0,  1],
])

# First adjust AnyGrasp grasp pose to tool axis convention, then project to base frame
# T_target_in_base = T_cam_in_base @ T_target_in_cam
T_target_in_base = T_cam_in_base @ (T_target_in_cam @ T_align)

# Output
print("Target pose in robot arm base coordinate system:")
print(T_target_in_base)
