"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
from yourdfpy import URDF

import pyroki as pk
import pyroki_snippets as pks


def main():
    """Main function for basic IK."""

    urdf = URDF.load("D:/aca/MM/vis_grasp/URDF/gripper/d1_description/urdf/d1_description.urdf")
    target_link_name = "Link6"

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # T = np.array([[-0.42842321, 0.76767154, 0.47658574, -0.05553023],
    #               [0.12879825, 0.57394007, -0.80870509, -0.05628628],
    #               [-0.89435147, -0.28508462, -0.34476404, -0.44114854],
    #               [0., 0., 0., 1.]])
    
    T = np.array([
        [0.18199572,  0.8636306,   0.47012744,  0.57031645],
        [0.25284308, -0.50313819,  0.82639114,  0.02724816],
        [0.95023579, -0.03153123, -0.30993202,  0.1938222 ],
        [0.        ,  0.        ,  0.        ,  1.        ]
    ])
    # T = np.linalg.inv(T)
    position = T[:3, 3]
    wxyz = Rotation.from_matrix(T[:3, :3]).as_quat(scalar_first=True)

    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=position, wxyz=wxyz
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        # Solve IK.
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )

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
