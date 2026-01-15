import numpy as np
import pickle
from SimpleHandEye.solvers import OpenCVSolver
import os
import glob
from handeye_utils import compute_camera_base_from_AX, save_npy

# Find calib folder: prefer latest under 'calib_results', otherwise ask user
print("Find calib folder: prefer latest under 'calib_results', otherwise ask user...")
base_calib_dir = 'calib_results'
if os.path.isdir(base_calib_dir):
    subdirs = [os.path.join(base_calib_dir, d) for d in os.listdir(base_calib_dir)
               if os.path.isdir(os.path.join(base_calib_dir, d))]
    if subdirs:
        latest = max(subdirs, key=os.path.getmtime)
        print(f"Found latest calib folder: {latest}")
        use_latest = input("Use this folder? [Y/n]: ").strip().lower() or 'y'
        if use_latest == 'n':
            data_folder = input("Enter calib folder path: ").strip()
        else:
            data_folder = latest
    else:
        data_folder = input("No calib subfolders found. Enter calib folder path: ").strip()
else:
    data_folder = input("calib_results not found. Enter calib folder path: ").strip()

# Normalize
data_folder = os.path.expanduser(data_folder)

def load_pkl_from_folder(filename):
    path = os.path.join(data_folder, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Expected file not found: {path}")
    with open(path, 'rb') as f:
        return pickle.load(f)

# -------------------------------
# Load data
# -------------------------------
A_list = load_pkl_from_folder('handeye_data_A.pkl')
B_list = load_pkl_from_folder('handeye_data_B.pkl')

# Try to find initial unfiltered result in the same folder
print("Try to find initial unfiltered result in the same folder...")
result_candidates = glob.glob(os.path.join(data_folder, 'handeye_result_unfiltered*.pkl'))
if result_candidates:
    result_path = result_candidates[0]
    print(f"Loading initial result from {result_path}")
    with open(result_path, 'rb') as f:
        result = pickle.load(f)
else:
    # fall back to asking user
    manual = input("No initial unfiltered result found in folder. Enter path to initial result pkl (or leave blank to abort): ").strip()
    if manual == '':
        raise FileNotFoundError("No initial result provided; cannot compute error norms.")
    with open(os.path.expanduser(manual), 'rb') as f:
        result = pickle.load(f)

X_init = result['X']
Y_init = result['Y']

# -------------------------------
# Compute error norms
# -------------------------------
def compute_error_norms(A_list, B_list, X, Y):
    """
    Compute Frobenius norm of residual AX - YB for each sample
    """
    errors = []
    for i, (A, B) in enumerate(zip(A_list, B_list)):
        res = A @ X - Y @ B
        err_norm = np.linalg.norm(res, ord='fro')
        errors.append(err_norm)
    return np.array(errors)

# -------------------------------
# Filter samples based on threshold
# -------------------------------
def filter_samples(A_list, B_list, error_norms, threshold):
    """
    Returns filtered A, B lists and indices where error_norm < threshold
    """
    indices = np.where(error_norms < threshold)[0]
    A_filtered = [A_list[i] for i in indices]
    B_filtered = [B_list[i] for i in indices]
    return A_filtered, B_filtered, indices

# -------------------------------
# Solve AX = YB using OpenCV solver
# -------------------------------
def solve_handeye(A_list, B_list):
    solver = OpenCVSolver(type='AX=YB')
    X, Y = solver.solve(A_list, B_list)
    return X, Y

# -------------------------------
# Main flow
# -------------------------------
if __name__ == "__main__":
    print(f"Loaded {len(A_list)} samples.")

    # Step 1: compute initial error norms
    error_norms = compute_error_norms(A_list, B_list, X_init, Y_init)
    print("Sorted error norms:", np.sort(error_norms))
    print("Initial error norms (sorted indices):", np.argsort(error_norms))
    print(f"Average initial error norm: {np.mean(error_norms):.6f}")

    # Step 2: ask user for threshold
    threshold = float(input("Enter error threshold to filter samples: "))

    # Step 3: filter samples
    A_filtered, B_filtered, filtered_indices = filter_samples(A_list, B_list, error_norms, threshold)
    print(f"{len(A_filtered)} samples remaining after threshold filtering.")

    # Step 4: recompute calibration
    if len(A_filtered) < 3:
        print("[ERROR] Not enough samples after filtering to recompute calibration (need at least 3).")
    else:
        X_new, Y_new = solve_handeye(A_filtered, B_filtered)
        print("\n=== Recomputed Hand-Eye Calibration Result ===")
        print("X (hand-to-eye transform):\n", X_new)
        print("Y (base-to-tag transform):\n", Y_new)
        cam_base = compute_camera_base_from_AX(A_filtered, X_new, B_list=B_filtered, Y_base_T_tag=Y_new)
        print("base_T_cam (derived, medoid over filtered samples):\n", cam_base.base_T_cam)
        print("cam_T_base (derived):\n", cam_base.cam_T_base)
        print(f"[INFO] base_T_cam selected filtered sample index: {cam_base.selected_index}")
        if cam_base.mean_fro_Ax_vs_YB is not None:
            print(
                f"[INFO] Consistency check ||(A@X) - (Y@B)||_F (B=tag_T_cam) (filtered): "
                f"mean={cam_base.mean_fro_Ax_vs_YB:.6f}, max={cam_base.max_fro_Ax_vs_YB:.6f}"
            )
        print("=============================================")

        # Optionally save new calibration
        out_path = os.path.join(data_folder, 'handeye_result_filtered.pkl')
        with open(out_path, 'wb') as f:
            pickle.dump(
                {
                    'X': X_new,  # hand_T_cam
                    'Y': Y_new,  # base_T_tag
                    'filtered_indices': filtered_indices,
                    'base_T_cam': cam_base.base_T_cam,
                    'cam_T_base': cam_base.cam_T_base,
                    'base_T_cam_selected_index': cam_base.selected_index,
                },
                f,
            )
        print(f"[INFO] Filtered calibration result saved to '{out_path}'.")

        # Save numpy outputs for convenient downstream loading
        print("[INFO] Saving numpy transforms...")
        print(" -", save_npy(data_folder, 'hand_to_eye_filtered.npy', X_new))
        print(" -", save_npy(data_folder, 'target_in_base_filtered.npy', Y_new))
        print(" -", save_npy(data_folder, 'camera_to_base_filtered.npy', cam_base.cam_T_base))
        print(" -", save_npy(data_folder, 'base_to_camera_filtered.npy', cam_base.base_T_cam))
