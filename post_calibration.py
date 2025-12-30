import numpy as np
import pickle
from SimpleHandEye.solvers import OpenCVSolver

# -------------------------------
# Load data
# -------------------------------
with open('handeye_data_A.pkl', 'rb') as f:
    A_list = pickle.load(f)

with open('handeye_data_B.pkl', 'rb') as f:
    B_list = pickle.load(f)

with open('handeye_result_unfiltered_dec-22.pkl', 'rb') as f:
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
        print("=============================================")

        # Optionally save new calibration
        with open('handeye_result_filtered.pkl', 'wb') as f:
            pickle.dump({'X': X_new, 'Y': Y_new, 'filtered_indices': filtered_indices}, f)
        print("[INFO] Filtered calibration result saved to 'handeye_result_filtered.pkl'.")
