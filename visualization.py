import time
import mujoco
import mujoco.viewer
import numpy as np
import os
from scipy.spatial.transform import Rotation
import cv2
from threading import Thread
import subprocess
import pinocchio as pin

ASSETS_PATH = 'assets'

class D1Sim:
    def __init__(self, 
                 mode='lowlevel', 
                 render=True, 
                 dt=0.002, 
                 height_map = None, 
                 xml_path=None,
                 camera_name = "front_camera", 
                 camera_resolution = (640, 480),
                 camera_depth_range = (0.35, 3.0), 
                 friction_model = None,
                 async_mode = False,
                 ):

        if xml_path is None:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH, 'mujoco/d1.xml')
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)

        if height_map is not None:
            try:
                self.updateHeightMap(height_map)
            except:
                raise Exception('Could not set height map. Are you sure the XML contains the required asset?')
        self.friction_model = friction_model
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1 / 60
        self.render_ds_ratio = max(1, _render_dt // dt)

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.render = True
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -45
            self.viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.renderer = None
        self.render = render
        self.step_counter = 0

        self.q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.show(self.q0)
        mujoco.mj_step(self.model, self.data)
        if self.render:
            self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))

    def show(self, q):
        self.data.qpos = q
        self.data.qvel = np.zeros_like(q)
        mujoco.mj_step(self.model, self.data)
        if self.render:
            self.viewer.sync()

    def get_servo_data(self, exec_path):
        """
        Calls the C++ executable that prints joint angles in degrees,
        and converts them to radians for Pinocchio.
        """
        while True:
            result = subprocess.run([exec_path], capture_output=True, text=True)
            if result.returncode != 0:
                print(f"C++ program failed: {result.stderr} re trying...")
            else:
                break

        line = result.stdout.strip()  # e.g., "13,-89.3,87.6,-9.6,16.3,-76.7,21"
        if not line:
            raise RuntimeError("C++ program returned empty output")

        # Split by comma
        q_deg = [float(x) for x in line.split(',')]
        # print(f"q_deg: {q_deg}")
        if len(q_deg) != 7:
            raise RuntimeError(f"Expected 7 joint angles, got {len(q_deg)}: {line}")

        q_rad = np.radians(q_deg)  # Convert degrees to radians
        # print(f"Joint angles (radians): {q_rad}")


        return np.array(q_rad), q_deg
    
    def set_joint_positions(self, q_rad_8):
        """
        Set joint positions from 8-element joint vector (radians)
        Order:
        [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, Joint7_1, Joint7_2]
        """
        assert len(q_rad_8) == self.model.nq

        self.data.qpos[:] = q_rad_8
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)



if __name__ == "__main__":
    sim = D1Sim(render=True)

    cpp_exe_path = "/home/doordash/Desktop/here_we_go_again/d1_arm/d1_sdk/build/get_joint_angle"

    update_period = 0.1  # seconds
    last_time = time.time()

    # print("Starting MuJoCo visualization loop...")

    while sim.viewer.is_running():
        now = time.time()

        if now - last_time >= update_period:
            try:
                q_rad, q_deg = sim.get_servo_data(cpp_exe_path)
                # Append gripper joints (locked)
                q_rad_8 = np.zeros(8)
                q_rad_8[:6] = q_rad[:6]   # Joint1–Joint6
                q_rad_8[6] = 0.0          # Joint7_1
                q_rad_8[7] = 0.0          # Joint7_2
                sim.set_joint_positions(q_rad_8)
                # print(q_rad_8)
                # print(f"Rendered joints (deg): {q_deg}")
            except Exception as e:
                print(f"Error: {e}")

            last_time = now

        # Keep physics stable even if we don’t update joints
        mujoco.mj_step(sim.model, sim.data)

        if sim.render:
            sim.viewer.sync()

        time.sleep(sim.dt)
