import os
import pinocchio as pin
import numpy as np
import time
import mujoco
import mujoco.viewer
import numpy as np
import os
from scipy.spatial.transform import Rotation
import cv2
from threading import Thread

ASSETS_PATH = 'assets'
class D1Model:
    def __init__(self):
        urdf_path = os.path.join(ASSETS_PATH, 'urdf/d1_description.urdf')
        urdf_root_path = os.path.join(ASSETS_PATH, 'urdf')
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, urdf_root_path)
        self.data = self.robot.data

    def forwardKinematics(self, q):
        self.robot.framesForwardKinematics(q)
        ef_frames = ['Link6']
        return {frame: self.robot.data.oMf[self.robot.model.getFrameId(frame)].homogeneous
                for frame in ef_frames}


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
