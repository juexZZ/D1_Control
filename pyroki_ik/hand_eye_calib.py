import cv2
import numpy as np
import glob
import os
from scipy.spatial.transform import Rotation as R
from urdfpy import URDF
from pupil_apriltags import Detector

# --- 参数设置 ---
URDF_PATH = r'D:\\aca\\MM\\vis_grasp\\机械臂URDF\\有夹爪版\d1_description\\urdf\\d1_description.urdf'
IMAGES_DIR = 'D:\\aca\\MM\\vis_grasp\\calib_data\\521\\images'  # 包含多张标定图像
ANGLES_FILE = 'D:\\aca\\MM\\vis_grasp\\calib_data\\521\\joint_angles.txt'  # 每行一个姿态，对应一张图像
TAG_SIZE = 0.1  # apriltag 边长（单位：米）
CAMERA_MATRIX = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]])
DIST_COEFFS = np.zeros((5,))

# --- 1. 从 URDF 构建 FK 求解器（手动） ---
def from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    return R

def build_fk_chain(urdf, base_link='base_link', end_link='Link6'):
    joints = []
    link = end_link
    while link != base_link:
        joint = next(j for j in urdf.joints if j.child == link)
        joints.insert(0, joint)
        link = joint.parent
    return joints

def compute_fk(joint_chain, joint_values):
    T = np.eye(4)
    for joint, q in zip(joint_chain, joint_values):
        T_origin = np.eye(4)
        T_origin[:3, :3] = from_rpy(*joint.origin.rpy)
        T_origin[:3, 3] = joint.origin.xyz

        if joint.joint_type == 'revolute':
            axis = joint.axis / np.linalg.norm(joint.axis)
            angle = q
            c, s = np.cos(angle), np.sin(angle)
            x, y, z = axis
            R_joint = np.array([
                [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
                [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)]
            ])
            T_joint = np.eye(4)
            T_joint[:3,:3] = R_joint
        elif joint.joint_type == 'prismatic':
            T_joint = np.eye(4)
            T_joint[:3, 3] = joint.axis * q
        else:
            T_joint = np.eye(4)

        T = T @ T_origin @ T_joint
    return T[:3,:3], T[:3,3]

# --- 2. 识别 Apriltag 并提取位姿 ---
def detect_apriltag_pose(img, tag_size, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detector = Detector(families='tag36h11')
    detections = detector.detect(gray, estimate_tag_pose=False)
    detections = [d for d in detections if d.tag_id == 42]
    if len(detections) == 0:
        raise RuntimeError("未检测到Apriltag")
    tag = detections[0]
    object_points = np.array([[-0.5, -0.5, 0],
                              [0.5, -0.5, 0],
                              [0.5,  0.5, 0],
                              [-0.5,  0.5, 0]]) * tag_size
    image_points = np.array(tag.corners, dtype=np.float32)
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    R_cam, _ = cv2.Rodrigues(rvec)
    t_cam = tvec.flatten()
    return R_cam, t_cam

# --- 主流程 ---

urdf = URDF.load(URDF_PATH)
joint_chain = build_fk_chain(urdf)
#print(joint_chain[1].parent)

with open(ANGLES_FILE, 'r') as f:
    all_joint_angles = [
        [np.deg2rad(float(angle)) for angle in line.strip().split()]
        for line in f.readlines()
    ]


image_paths = sorted(glob.glob(os.path.join(IMAGES_DIR, '*.png')))
print(image_paths)

R_gripper2base_list = []
t_gripper2base_list = []
R_target2cam_list = []
t_target2cam_list = []

for img_path, joint_angles in zip(image_paths, all_joint_angles):
    img = cv2.imread(img_path)
    R_cam, t_cam = detect_apriltag_pose(img, TAG_SIZE, CAMERA_MATRIX, DIST_COEFFS)
    R_grip, t_grip = compute_fk(joint_chain, joint_angles)

    R_target2cam_list.append(R_cam)
    t_target2cam_list.append(t_cam)
    R_gripper2base_list.append(R_grip)
    t_gripper2base_list.append(t_grip)

# --- 3. 执行手眼标定 (eye-to-hand) ---
R_cam2base, t_cam2base = cv2.calibrateHandEye(
    R_gripper2base_list, t_gripper2base_list,
    R_target2cam_list, t_target2cam_list,
    method=cv2.CALIB_HAND_EYE_TSAI)

print("Camera to Base (Rotation):\n", R_cam2base)
print("Camera to Base (Translation):\n", t_cam2base)
print("T_base_cam:\n", np.vstack([np.hstack([R_cam2base, np.array(t_cam2base).reshape(3, 1)]), [0, 0, 0, 1]]))