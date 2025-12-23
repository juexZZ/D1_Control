import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# 连接到物理引擎（开启可视化界面）
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加默认模型路径

# 加载地面和机械臂模型
p.setGravity(0, 0, -9.8)
# plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("D:\\aca\\MM\\vis_grasp\\URDF\\gripper\\d1_description\\urdf\\d1_description.urdf", [0, 0, 0], useFixedBase=True)

# 设置机械臂末端连杆索引（Empty_Link6 是第6个旋转关节后的连杆）
# 注意：PyBullet中连杆索引需要根据加载顺序确定，可通过 p.getNumJoints() 和 p.getJointInfo() 查询
# 手动确定Empty_Link6对应的索引（此处假设为7，具体需根据实际加载顺序调整）
# p.getNumJoints()
# p.getJointInfo()
end_effector_link_index = 7  # 需根据实际情况修改！

# 目标在机械臂基座坐标系下的变换矩阵
# T = np.array([
#     [-0.19678868, -0.01845381, -0.98027222,  0.0496546],
#     [ 0.94862631, -0.25623472, -0.18561212, -0.04436804],
#     [-0.2477545 , -0.9664384 ,  0.06792988, -0.38156981],
#     [ 0.        ,  0.        ,  0.        ,  1.        ]
# ])

T = np.array([[-0.42842321,  0.76767154,  0.47658574, -0.05553023],
 [ 0.12879825,  0.57394007, -0.80870509, -0.05628628],
 [-0.89435147, -0.28508462, -0.34476404, -0.44114854],
 [ 0.,         0.,          0.,          1.        ]])
target_pos = T[:3, 3].tolist()
target_orn = R.from_matrix(T[:3, :3]).as_quat().tolist()  # [x, y, z, w]

# Revolute joints 限制（从 URDF 解析得出）
lower_limits = [-2.35, -1.57, -1.57, -2.35, -1.57, -2.35]
upper_limits = [ 2.35,  1.57,  1.57,  2.35,  1.57,  2.35]
joint_ranges = [u - l for u, l in zip(upper_limits, lower_limits)]
rest_poses = [(u + l) / 2 for u, l in zip(upper_limits, lower_limits)]  # 中间位姿作为参考

# 计算逆运动学（IK）
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
    residualThreshold=1e-8,
)

# 获取关节数量并过滤旋转关节（排除夹爪的棱柱关节）
num_joints = p.getNumJoints(robot_id)
active_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    if joint_info[2] == p.JOINT_REVOLUTE:  # 仅处理旋转关节（Joint1-Joint6）
        active_joints.append(i)

# 提取前6个旋转关节的角度（假设Joint1-Joint6是前6个关节）
solution = joint_angles[:6]  # 6个旋转关节的角度

# 打印结果
print("计算得到的关节角度（弧度）：")
for i, angle in enumerate(solution):
    print(f"Joint{i+1}: {np.round(angle, 4)} rad")

# 应用角度到机械臂
for i in active_joints[:6]:  # 仅应用到前6个旋转关节
    p.resetJointState(robot_id, i, solution[i])

# 运行仿真并观察结果
for _ in range(100000):
    p.stepSimulation()
    time.sleep(1/240)

end_state = p.getLinkState(robot_id, end_effector_link_index)
actual_pos = end_state[0]
actual_orn = end_state[1]
print("实际末端位置：", np.round(actual_pos, 3))
print("目标末端位置：", target_pos)

# 断开连接
p.disconnect()

