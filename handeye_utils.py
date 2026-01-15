import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


def _as_homogeneous(T: np.ndarray, name: str) -> np.ndarray:
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got shape {T.shape}")
    return T


def invert_homogeneous(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 rigid transform."""
    T = _as_homogeneous(T, "T")
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=float)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def compute_base_T_cam_samples(A_list: List[np.ndarray], X_hand_T_cam: np.ndarray) -> List[np.ndarray]:
    """
    Given A_i = base_T_hand and X = hand_T_cam, compute base_T_cam_i = A_i @ X for each sample.
    """
    X_hand_T_cam = _as_homogeneous(X_hand_T_cam, "X_hand_T_cam")
    out: List[np.ndarray] = []
    for i, A in enumerate(A_list):
        A = _as_homogeneous(A, f"A_list[{i}]")
        out.append(A @ X_hand_T_cam)
    return out


def compute_base_T_cam_samples_from_YB(
    B_list: List[np.ndarray],
    Y_base_T_tag: np.ndarray,
) -> List[np.ndarray]:
    """
    Alternative derivation using Y and B (from AX=YB).

    With this repo's convention:
      - Y = base_T_tag
      - B = tag_T_cam
    so:
      - base_T_cam = Y @ B
    """
    Y_base_T_tag = _as_homogeneous(Y_base_T_tag, "Y_base_T_tag")
    out: List[np.ndarray] = []
    for i, B in enumerate(B_list):
        B = _as_homogeneous(B, f"B_list[{i}]")
        out.append(Y_base_T_tag @ B)
    return out


def _fro_distance(T1: np.ndarray, T2: np.ndarray) -> float:
    return float(np.linalg.norm(T1 - T2, ord="fro"))


def select_medoid_transform(T_list: List[np.ndarray]) -> Tuple[int, np.ndarray]:
    """
    Pick a robust representative transform using the Frobenius-distance medoid:
    choose the T_k that minimizes sum_j ||T_k - T_j||_F.
    """
    if len(T_list) == 0:
        raise ValueError("T_list is empty")
    if len(T_list) == 1:
        return 0, _as_homogeneous(T_list[0], "T_list[0]")

    Ts = [_as_homogeneous(T, f"T_list[{i}]") for i, T in enumerate(T_list)]
    costs = []
    for i, Ti in enumerate(Ts):
        costs.append(sum(_fro_distance(Ti, Tj) for Tj in Ts))
    idx = int(np.argmin(costs))
    return idx, Ts[idx]


@dataclass(frozen=True)
class CameraBaseResult:
    # Selected representative transforms
    base_T_cam: np.ndarray
    cam_T_base: np.ndarray
    # Diagnostics
    base_T_cam_samples: List[np.ndarray]
    selected_index: int
    # Optional self-check metrics (if available)
    mean_fro_Ax_vs_YB: float | None = None
    max_fro_Ax_vs_YB: float | None = None


def compute_camera_base_from_AX(
    A_list: List[np.ndarray],
    X_hand_T_cam: np.ndarray,
    *,
    B_list: List[np.ndarray] | None = None,
    Y_base_T_tag: np.ndarray | None = None,
) -> CameraBaseResult:
    """
    Compute camera_to_base using the solved X (hand_T_cam) and the recorded A_i (base_T_hand).
    input:
    - A_list: list of base_T_hand
    - X_hand_T_cam: hand_T_cam
    - B_list: list of tag_T_cam (as currently saved by `handeye_calib.py`, since it stores inv(cam_T_tag))
    - Y_base_T_tag: base_T_tag
    output:
    - base_T_cam_i = A_i @ X
    - base_T_cam = robust medoid over i
    - cam_T_base = inv(base_T_cam)
    """
    base_T_cam_samples = compute_base_T_cam_samples(A_list, X_hand_T_cam)
    selected_index, base_T_cam = select_medoid_transform(base_T_cam_samples)
    cam_T_base = invert_homogeneous(base_T_cam)

    mean_fro_yb = None
    max_fro_yb = None
    if B_list is not None and Y_base_T_tag is not None and len(B_list) == len(A_list):
        # With our convention B = tag_T_cam, base_T_cam should be Y @ B.
        base_T_cam_samples_yb = compute_base_T_cam_samples_from_YB(B_list, Y_base_T_tag)
        diffs_yb = [
            _fro_distance(T_ax, T_yb) for T_ax, T_yb in zip(base_T_cam_samples, base_T_cam_samples_yb)
        ]
        if diffs_yb:
            mean_fro_yb = float(np.mean(diffs_yb))
            max_fro_yb = float(np.max(diffs_yb))

    return CameraBaseResult(
        base_T_cam=base_T_cam,
        cam_T_base=cam_T_base,
        base_T_cam_samples=base_T_cam_samples,
        selected_index=selected_index,
        mean_fro_Ax_vs_YB=mean_fro_yb,
        max_fro_Ax_vs_YB=max_fro_yb,
    )


def save_npy(out_dir: str, filename: str, arr: np.ndarray) -> str:
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    np.save(path, np.asarray(arr, dtype=float))
    return path

