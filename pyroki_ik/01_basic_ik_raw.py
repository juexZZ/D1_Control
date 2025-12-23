"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
from yourdfpy import URDF
from spatialmath import SE3

import pyroki as pk
import pyroki_snippets as pks
from scipy.spatial.transform import Rotation as R


def main():
    # anygrasp到基座pose转换

    # 目标在相机坐标系下的位姿
    T_target_in_cam = np.load("D:\\aca\MM\\vis_grasp\\best_anygrasp_pose.npy")
    print("目标在相机坐标系下的位姿:")
    print(T_target_in_cam)

    # 相机在机械臂基座下的位姿

    T_cam_in_base = np.array([
        [ 0.01854991, -0.22026411,  0.97526387,  0.36235041],
        [-0.99972266,  0.01006837,  0.02128908, -0.00776866],
        [-0.01450853, -0.97538831, -0.22001626,  0.02456633],
        [ 0.        ,  0.        ,  0.        ,  1.        ],
    ])

    # 轴对齐转换
    T_align = np.array([
        [0,  0,  1,  0],
        [0,  -1,  0,  0],
        [1,  0,  0,  0],
        [0,  0,  0,  1],
    ])

    # 绕y轴逆时针旋转90°
    T_y = np.array([
        [ 0, 0,  1, 0],
        [ 0, 1,  0, 0],
        [-1, 0,  0, 0],
        [ 0, 0,  0, 1],
    ])

    # # 绕y轴顺时针旋转90°
    # T_y = np.array([
    #     [ 0, 0, -1, 0],
    #     [ 0, 1,  0, 0],
    #      [1, 0,  0, 0],
    #     [ 0, 0,  0, 1],
    # ])

    # 先把anygrasp抓取姿态调整到工具轴约定下，再投影到基座系
    # T_target_in_base = T_cam_in_base @ T_target_in_cam
    # T_target_in_base = T_cam_in_base @ (T_target_in_cam @ T_align)
    T_target_in_base = (T_cam_in_base @ T_target_in_cam) @ T_y

    # 输出
    print("目标在机械臂基座坐标系下的位姿:")
    print(T_target_in_base)

    """Main function for basic IK."""

    urdf = URDF.load("D:/aca/MM/vis_grasp/URDF/gripper/d1_description/urdf/d1_description.urdf")
    target_link_name = "Link_tcp"

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    T_t2b = T_target_in_base

    # 提取 position（位移向量）
    position = T_t2b[:3, 3]

    # 提取旋转矩阵
    rotation_matrix = T_t2b[:3, :3]

    # 转换为四元数（xyzw）
    r = R.from_matrix(rotation_matrix)
    quat_xyzw = r.as_quat()  # [x, y, z, w]

    # 转换为 viser 需要的格式（wxyz）
    wxyz = np.roll(quat_xyzw, 1)  # [w, x, y, z]

    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=position, wxyz=wxyz
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)
    
    cnt = 0
    while True:
        # Solve IK.
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )
        if cnt == 100:
            print("Joint Angles (deg):", np.rad2deg(solution))
            deg_solution = np.rad2deg(solution)
            deg_solution[0] = -deg_solution[0]
            deg_solution[3] = -deg_solution[3]
            deg_solution = deg_solution[:7]
            deg_solution[6] = 60
            print("Transformed D1 joint angles:", repr(deg_solution))
            
        cnt += 1

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)

    from d1py.interface import D1Arm
    robot = D1Arm()
    solution_deg = np.rad2deg(solution)
    robot.set_all_joints(solution_deg, mode=1)  # FIXME: 7 joints, but 8 in URDF


if __name__ == "__main__":
    main()
