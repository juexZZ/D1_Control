import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# def draw_pose(pos, orn, length=0.1):
#     """在给定位置绘制姿态坐标轴（X红，Y绿，Z蓝）"""
#     rot = R.from_quat(orn).as_matrix()
#     x_axis = pos + rot[:, 0] * length
#     y_axis = pos + rot[:, 1] * length
#     z_axis = pos + rot[:, 2] * length

#     p.addUserDebugLine(pos, x_axis, [1, 0, 0], lineWidth=3, lifeTime=0)  # X - 红
#     p.addUserDebugLine(pos, y_axis, [0, 1, 0], lineWidth=3, lifeTime=0)  # Y - 绿
#     p.addUserDebugLine(pos, z_axis, [0, 0, 1], lineWidth=3, lifeTime=0)  # Z - 蓝

def draw_pose(pos, orn, length=0.1, label=""):
    """绘制某个位置的姿态坐标系（X红，Y绿，Z蓝）"""
    rot = R.from_quat(orn).as_matrix()
    x_axis = pos + rot[:, 0] * length
    y_axis = pos + rot[:, 1] * length
    z_axis = pos + rot[:, 2] * length

    p.addUserDebugLine(pos, x_axis, [1, 0, 0], lineWidth=3, lifeTime=0)  # X - 红
    p.addUserDebugLine(pos, y_axis, [0, 1, 0], lineWidth=3, lifeTime=0)  # Y - 绿
    p.addUserDebugLine(pos, z_axis, [0, 0, 1], lineWidth=3, lifeTime=0)  # Z - 蓝

    if label:
        p.addUserDebugText(label, pos, textColorRGB=[1, 1, 1], textSize=1.2, lifeTime=0)


# 初始化仿真
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# # 转换base坐标轴朝向
# rot_mat = np.array([
#     [0, 0, 1],  # X_new = -Y_old
#     [1, 0, 0],  # Y_new =  Z_old
#     [0, 1, 0]   # Z_new =  X_old
# ])
# quat_base = R.from_matrix(rot_mat).as_quat()  # xyzw

# robot_id = p.loadURDF(
#     "D:/aca/MM/vis_grasp/URDF/gripper/d1_description/urdf/d1_description.urdf",
#     basePosition=[0, 0, 0],
#     baseOrientation=quat_base,
#     useFixedBase=True
# )

# 加载 URDF
robot_id = p.loadURDF("D:/aca/MM/vis_grasp/URDF/gripper/d1_description/urdf/d1_description.urdf",
                      basePosition=[0, 0, 0], useFixedBase=True)

#可视化base坐标轴
base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
draw_pose(base_pos, base_orn, length=0.15, label="base")

# 获取关节总数
num_joints = p.getNumJoints(robot_id)

# 提取每个 joint 的限制
lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

revolute_joint_indices = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_type = joint_info[2]
    joint_name = joint_info[1].decode('utf-8')

    if joint_type == p.JOINT_REVOLUTE:
        lower = joint_info[8]
        upper = joint_info[9]
        revolute_joint_indices.append(i)
    elif joint_type == p.JOINT_PRISMATIC:
        lower = joint_info[8]
        upper = joint_info[9]
    else:
        lower = 0.0
        upper = 0.0

    range_ = upper - lower
    rest = (upper + lower) / 2

    lower_limits.append(lower)
    upper_limits.append(upper)
    joint_ranges.append(range_)
    rest_poses.append(rest)

# 设置目标末端位姿
# T = np.array([
#     [-0.19678868, -0.01845381, -0.98027222,  0.0496546],
#     [ 0.94862631, -0.25623472, -0.18561212, -0.04436804],
#     [-0.2477545 , -0.9664384 ,  0.06792988, -0.38156981],
#     [ 0.        ,  0.        ,  0.        ,  1.        ]
# ])

# T = np.array([[-0.42842321,  0.76767154,  0.47658574, -0.05553023],
#  [ 0.12879825,  0.57394007, -0.80870509, -0.05628628],
#  [-0.89435147, -0.28508462, -0.34476404, -0.44114854],
#  [ 0.,         0.,          0.,          1.        ]])

# T = np.array([
#     [ 0.18199572,  0.8636306,   0.47012744,  0.60930539],
#     [ 0.25284308, -0.50313819,  0.82639114,  0.02724816],
#     [ 0.95023579, -0.03153123, -0.30993202,  0.1938222 ],
#     [ 0.,          0.,          0.,          1.        ]
# ])

# T = np.array([
#     [ 0.49694072,  0.86693829, -0.03739185,  0.52724835],
#     [ 0.42049619, -0.20290474,  0.88431472, -0.04708666],
#     [ 0.75908558, -0.45519205, -0.46539215, -0.01263643],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])

# T = np.array([
#     [ 0.88312996,  0.37282884, -0.28462350,  0.53633840],
#     [ 0.30460833, -0.91731383, -0.25641582, -0.03847883],
#     [-0.35670168,  0.13975429, -0.92370622,  0.01839721],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])

# T = np.array([
#     [ 0.96036361, -0.24375348,  0.13496777,  0.52949249],
#     [ 0.15039553,  0.86130733,  0.48531518, -0.01226171],
#     [-0.23455520, -0.44579586,  0.86385788,  0.00132492],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])

# T = np.array([
#     [ 0.96495562, -0.25585093,  0.05831742,  0.60486620],
#     [ 0.18824011,  0.82972981,  0.52546557, -0.00686614],
#     [-0.18282856, -0.49607330,  0.84881386, -0.01640866],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])

T = np.array([
    [ 0.43637456,  0.89401240, -0.10158271,  0.57274610],
    [ 0.03693169,  0.09500712,  0.99479124, -0.01575422],
    [ 0.89900688, -0.43785323,  0.00844130,  0.01122792],
    [ 0.        ,  0.        ,  0.        ,  1.        ],
])


# T = np.array(
#    [[ 0.24114924, -0.96073396, -0.1372487,  -0.03584759],
#  [ 0.39722483, -0.03132214,  0.91718658,  0.00881749],
#  [-0.88547131, -0.27569745,  0.37407408, -0.37684493],
#  [ 0. ,         0.,          0. ,         1.        ]] 
# )

# T = np.array([[1,  0,  0, 0],
#  [0,  1,  0, 1],
#  [0,  0,  1, 0],
#  [0,  0,  0, 1],])

target_pos = T[:3, 3].tolist()
target_orn = R.from_matrix(T[:3, :3]).as_quat().tolist()
# 可视化目标末端姿态
draw_pose(np.array(target_pos), target_orn, length=0.1, label="anygrasp_in_base")

# 目标位姿矩阵 T (旧base坐标系定义的)
T_old = T.copy()

# import sympy as sp
# R_y = sp.Matrix([[0,0,1],[0,1,0],[-1,0,0]])
# R_x = sp.Matrix([[1,0,0],[0,0,1],[0,-1,0]])
# R_old2pb = R_x*R_y

# # 旧base → PyBullet base 的旋转矩阵
# R_old2pb = np.array([
#     [0, 0, 1],
#     [1, 0, 0],
#     [0, 1, 0]
# ])

# # 构造 4x4 齐次变换矩阵
# T_old2pb = np.eye(4)
# T_old2pb[:3, :3] = R_old2pb

# T_old2pb = np.array([[0,  0,  1, 0],
#  [-1,  0,  0, 0],
#  [0,  -1,  0, 0],
#  [0,  0,  0, 1],])

# T_new = T_old2pb @ T_old
# #T_new =  T_old @ T_old2pb

# target_pos = T_new[:3, 3].tolist()
# target_orn = R.from_matrix(T_new[:3, :3]).as_quat().tolist()

# # 可视化新目标姿态（在 pybullet base 坐标系下）
# draw_pose(np.array(target_pos), target_orn, length=0.1, label="target_in_pb_base")

# 可视化相机在base下的位置
# R_urdf_to_pb = np.array([
#     [0,  0, 1],   # Z_urdf -> X_pb
#     [1,  0, 0],   # X_urdf -> Y_pb
#     [0,  1, 0],   # Y_urdf -> Z_pb
# ])
# T_cam_in_base = np.array( [[ 0.9953148,   0.08838702, -0.03919424, -0.01876423],
#  [ 0.09080951, -0.71537161,  0.69281822, -0.16431749],
#  [ 0.03319769, -0.69313144, -0.72004633, -0.25421669],
#  [ 0.,          0.,          0.,          1.        ]])

# T_cam_in_base = np.array([
#     [-0.02747287,  0.58922555,  0.80750139,  0.41898894],
#     [-0.99956001, -0.0252285,  -0.01559812, -0.0039617 ],
#     [ 0.01118124, -0.80757463,  0.58965939,  0.0671422 ],
#     [ 0.,          0.,          0.,          1.        ]
# ])

# T_cam_in_base = T_cam_in_base = np.array([
#     [ 0.27447326,  0.08208445,  0.95804850,  0.34933665],
#     [-0.94093900, -0.18247626,  0.28519505, -0.06072691],
#     [ 0.19823782, -0.97977810,  0.02715160,  0.01727595],
#     [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
# ])

# T_cam_in_base = np.array([
#     [ 0.30553955,  0.15104994,  0.94012207,  0.42653559],
#     [-0.91755487, -0.21712559,  0.33309090, -0.06350849],
#     [ 0.25443792, -0.96438602,  0.07225615, -0.00921762],
#     [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
# ])

# T_cam_in_base = np.array([
#     [ 0.09321143,  0.11136147,  0.98939894,  0.37604502],
#     [-0.99326275, -0.05831954,  0.10013958, -0.02880620],
#     [ 0.06885298, -0.99206726,  0.10517515,  0.01234557],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])
# T_cam_in_base = np.array([
#     [ 0.05393264,  0.03654861,  0.99787548,  0.39873969],
#     [-0.99641561, -0.06325194,  0.05617043, 0.00211961],
#     [ 0.06517051, -0.99732813,  0.03300626, 0.02055922],
#     [ 0.        ,  0.        ,  0.        ,  1.        ],
# ])

T_cam_in_base = np.array([
    [ 0.01854991, -0.22026411,  0.97526387,  0.36235041],
    [-0.99972266,  0.01006837,  0.02128908, -0.00776866],
    [-0.01450853, -0.97538831, -0.22001626,  0.02456633],
    [ 0.        ,  0.        ,  0.        ,  1.        ],
])


T_cam = T_cam_in_base[:3, 3].tolist()
R_cam = R.from_matrix(T_cam_in_base[:3, :3]).as_quat().tolist()
# R_cam_in_base = T_cam_in_base[:3, :3]
# t_cam_in_base = T_cam_in_base[:3, 3]
# R_cam_in_pb_world = R_urdf_to_pb @ R_cam_in_base
# t_cam_in_pb_world = R_urdf_to_pb @ t_cam_in_base
# T_cam = t_cam_in_pb_world.tolist()
# R_cam = R.from_matrix(R_cam_in_pb_world).as_quat().tolist()
draw_pose(np.array(T_cam), R_cam, length=0.1, label="cam_in_base")


# 获取末端 link 的索引（请确认是否为 link6）
end_effector_link_index = 7  # 如果不确定可以用 p.getJointInfo(...) 查
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint Index: {i}, Joint Name: {info[1].decode('utf-8')}, Link Name: {info[12].decode('utf-8')}")


# 执行 IK
joint_angles = p.calculateInverseKinematics(
    robot_id,
    end_effector_link_index,
    target_pos,
    target_orn,
    lowerLimits=lower_limits,
    upperLimits=upper_limits,
    jointRanges=joint_ranges,
    restPoses=rest_poses,
    maxNumIterations=100000,
    residualThreshold=1e-6
)

# 提取前6个 revolute 关节角度
solution = [joint_angles[i] for i in revolute_joint_indices[:6]]

# 输出结果
print("计算得到的关节角度（弧度）：")
for i, angle in enumerate(solution):
    name = p.getJointInfo(robot_id, revolute_joint_indices[i])[1].decode('utf-8')
    lo = lower_limits[revolute_joint_indices[i]]
    hi = upper_limits[revolute_joint_indices[i]]
    flag = "✅" if lo <= angle <= hi else "❌"
    print(f"{flag} {name}: {angle:.4f} rad ({np.degrees(angle):.2f}°)  ∈ [{np.degrees(lo):.1f}°, {np.degrees(hi):.1f}°]")

# 应用角度
revolute_joint_indices = [0, 0, 0, 0, 0, 0]
for i, angle in zip(revolute_joint_indices[:6], solution):
    p.resetJointState(robot_id, i, angle)

# 仿真运行
for _ in range(100000):
    p.stepSimulation()
    time.sleep(1/240)

# 末端验证
end_state = p.getLinkState(robot_id, end_effector_link_index)
actual_pos = end_state[0]
print("实际末端位置：", np.round(actual_pos, 3))
print("目标末端位置：", np.round(target_pos, 3))

p.disconnect()
