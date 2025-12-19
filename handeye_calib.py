from SimpleHandEye.interfaces.cameras import RealSenseCamera
from SimpleHandEye.interfaces.apriltag import ApriltagTracker
from SimpleHandEye.solvers import OpenCVSolver
from pinocchio.robot_wrapper import RobotWrapper
import cv2
import pinocchio as pin
import numpy as np
import subprocess
import os

# -----------------------------------------
# Camera callback to show color images
# -----------------------------------------
def showImage(color_frame, depth_frame, ir1_frame, ir2_frame):
    cv2.imshow('Camera', color_frame)
    cv2.waitKey(1)  # short wait to allow image update

# -----------------------------------------
# Get joint angles from your C++ D1 SDK program
# -----------------------------------------
def get_joint_angles_from_cpp(exe_path="/home/doordash/Desktop/.../get_arm_joint_angle"):
    """
    Calls the C++ executable that prints joint angles in degrees,
    and converts them to radians for Pinocchio.
    """
    while True:
        result = subprocess.run([exe_path], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"C++ program failed: {result.stderr} re trying...")
        else:
            break

    line = result.stdout.strip()  # e.g., "13,-89.3,87.6,-9.6,16.3,-76.7,21"
    if not line:
        raise RuntimeError("C++ program returned empty output")

    # Split by comma
    q_deg = [float(x) for x in line.split(',')]
    print(f"q_deg: {q_deg}")
    if len(q_deg) != 7:
        raise RuntimeError(f"Expected 7 joint angles, got {len(q_deg)}: {line}")

    q_rad = np.radians(q_deg)  # Convert degrees to radians
    print(f"Joint angles (radians): {q_rad}")
    return np.array(q_rad), q_deg


# -----------------------------------------
# Get gripper pose using URDF and Pinocchio
# -----------------------------------------
class D1Model:
    def __init__(self, urdf_path=None, urdf_root_path=None):
        if urdf_path is None:
            urdf_path = os.path.join(ASSETS_PATH, 'urdf/d1_description.urdf')
        if urdf_root_path is None:
            urdf_root_path = os.path.join(ASSETS_PATH, 'urdf')
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, urdf_root_path)
        self.data = self.robot.data

    def forwardKinematics(self, q):
        self.robot.framesForwardKinematics(q)
        ef_frames = ['Link6']
        return {frame: self.robot.data.oMf[self.robot.model.getFrameId(frame)].homogeneous
                for frame in ef_frames}

    def getGripperPose(self, cpp_exe_path):                                # last joint (gripper/fixed) set to zero

        # Fetch live joint angles
        q_sdk, q_sdk_deg = get_joint_angles_from_cpp(cpp_exe_path)  # length 7
        print(f"q_sdk: {q_sdk}")
        # Use only the first 6 joints (arm)
        print(f"robot model nq: {self.robot.model.nq}") # 8
        q = np.zeros(8)
        q[:6] = q_sdk[:6]

        # q = np.zeros(self.robot.model.nq)                            # length 8
        # q[:7] = q_sdk                                     # first 7 joints from SDK
        # q[7] = 0.0                                       # last joint (gripper/fixed) set to zero

        # Forward kinematics
        # pin.forwardKinematics(model, data, q)
        fks = self.forwardKinematics(q)
        # model.updateFramePlacements(model, data)
        
        # End-effector frame
        ee_frame_name = "Link6"  # Replace with your URDF EE frame
        bae_T_ee_homo = fks[ee_frame_name]

        # Convert SE3 to 4x4 numpy array for the solver

        return bae_T_ee_homo, q_sdk, q_sdk_deg
# -----------------------------------------
# Take a sample: append A (hand) and B (tag)
# -----------------------------------------
def take_sample(camera, tag_pose_tracker, solver, d1_model, cpp_exe_path,
                A_list, B_list, apriltag_info, apriltag_imgs_raw, q_rad_list, q_deg_list):
    print("[INFO] Taking sample...")

    # A = hand pose
    A, q_rad, q_deg = d1_model.getGripperPose(cpp_exe_path)

    print("hand pose", A)
    q_rad_list.append(q_rad)
    q_deg_list.append(q_deg)

    # Image from camera
    img = camera.color_frame.copy()

    # B = tag pose (tag_T_cam)
    info = tag_pose_tracker.getPoseAndCorners(img, tag_id=42)
    if info is None:
        print("[WARN] Tag not detected, skipping sample")
        return

    B = np.linalg.inv(info['pose'])
    print("AprilTag pose (tag_T_cam):\n", info['pose'])

    # Save data
    A_list.append(A)
    B_list.append(B)
    apriltag_info.append(info)
    apriltag_imgs_raw.append(img)

    print("Sample taken.")
    print(f"Total samples: {len(A_list)}")
    print("-" * 40)

# -----------------------------------------
# Compute hand-eye calibration
# -----------------------------------------
def compute_handeye(solver, A_list, B_list):
    print("[INFO] Computing hand-eye calibration...")
    if len(A_list) < 3:
        print("[ERROR] Not enough samples (need at least 3)")
        return
    
    try:
        X, Y = solver.solve(A_list, B_list)
        print("\n=== Hand-Eye Calibration Result ===")
        print("X (hand-to-eye transform):\n", X)
        print("Y (base-to-tag transform):\n", Y)
        print("==================================\n")
    except Exception as e:
        print("[ERROR] Calibration failed:", e)
        A_list.clear()
        B_list.clear()

# -----------------------------------------
# Main program
# -----------------------------------------
import pickle
ASSETS_PATH = 'assets'
def main():
    urdf_path = os.path.join(ASSETS_PATH, 'urdf/d1_description.urdf')  # <-- Set your URDF path here
    urdf_root_path = os.path.join(ASSETS_PATH, 'urdf')
    cpp_exe_path = "/home/doordash/Desktop/here_we_go_again/d1_arm/d1_sdk/build/get_joint_angle"     # <-- Path to your compiled D1 C++ program
    d1_model = D1Model(urdf_path, urdf_root_path)
    # Initialize camera
    camera = RealSenseCamera(callback_fn=showImage)

    # Get intrinsic parameters
    intrinsics = camera.getIntrinsics()
    K = intrinsics['RGB']['K']
    D = intrinsics['RGB']['D']
    print("Camera Intrinsic Matrix (K):\n", K)
    print("Distortion Coefficients (D):\n", D)

    # Initialize AprilTag tracker
    tag_pose_tracker = ApriltagTracker(
        tag_size=0.07614,  # Set your tag size (meters)
        intrinsic_matrix=K,
        distortion_coeffs=D
    )


    # Initialize solver

    solver = OpenCVSolver(type='AX=YB')

    # Data containers
    A_list = []
    B_list = []
    apriltag_info = []
    apriltag_imgs_raw = []
    q_rad_list = []
    q_deg_list = []

    print("\n--- Hand–Eye Calibration ---")
    print("Press:")
    print("  s -> take sample")
    print("  c -> compute calibration")

    print("  q -> quit")
    print("----------------------------\n")

    while True:
        key = input("Enter command [s/c/q]: ").strip().lower()
        if key == 's':
            take_sample(camera, tag_pose_tracker, solver, d1_model, cpp_exe_path,
                        A_list, B_list, apriltag_info, apriltag_imgs_raw, q_rad_list, q_deg_list)
        elif key == 'c':
            print("[INFO]   Number of samples:", len(A_list), len(B_list))
            print("[INFO] saving A and B lists to handeye_data_A.pkl and handeye_data_B.pkl")
            with open('handeye_data_A.pkl', 'wb') as f:
                pickle.dump(A_list, f)
            with open('handeye_data_B.pkl', 'wb') as f:
                pickle.dump(B_list, f)
            print("[INFO] saving april tag image raw data to apriltag_imgs_raw.pkl")
            with open('apriltag_imgs_raw.pkl', 'wb') as f:
                pickle.dump(apriltag_imgs_raw, f)
            print("[INFO] saving joint angles (rad) to q_rad_list.pkl and (deg) to q_deg_list.pkl")

            with open('q_rad_list.pkl', 'wb') as f:
                pickle.dump(q_rad_list, f)
            with open('q_deg_list.pkl', 'wb') as f:
                pickle.dump(q_deg_list, f)

            compute_handeye(solver, A_list, B_list)
        elif key == 'q':
            print("[INFO] Exiting.")
            break
        else:
            print("[WARN] Unknown command")

if __name__ == "__main__":
    main()
