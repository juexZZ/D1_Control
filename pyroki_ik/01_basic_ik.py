"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
import viser
from viser.extras import ViserUrdf
from yourdfpy import URDF

import pyroki as pk
import pyroki.pyroki_snippets as pks


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

    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.61, 0.0, 0.56), wxyz=(0, 0, 1, 0)
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
        print(solution)

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
