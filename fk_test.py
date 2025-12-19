# fk test, non interactive
import os
import pinocchio as pin
import numpy as np
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

model = D1Model()

print(model.forwardKinematics(np.zeros(8))['Link6'])

# print(model.forwardKinematics(np.zeros(8))['Link6'].rotation)
# print(model.forwardKinematics(np.zeros(8))['Link6'].translation)