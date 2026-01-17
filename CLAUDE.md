# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

D1_Control is a robotic arm control system for the D1 robot arm, integrating:
- Hand-eye calibration (camera-to-robot-base transformation)
- Forward and inverse kinematics (FK/IK)
- MuJoCo physics simulation
- Real robot integration via C++ D1 SDK

## Setup

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/juexZZ/D1_Control.git

# Update submodules (if already cloned)
git submodule update --init --recursive

# Install pytorch_kinematics (editable mode)
cd pytorch_kinematics && pip install -e .

# Install PyRoKi (editable mode)
cd ../pyroki_ik/pyroki && pip install -e .

# Activate conda environment
conda activate hand-eye
```

**Key dependencies:** pinocchio, mujoco, numpy, scipy, opencv-python, SimpleHandEye, jax, pybullet, open3d

## Testing

```bash
# Run pytorch_kinematics tests
cd pytorch_kinematics && pytest tests/

# Test forward kinematics
python fk_test.py
```

## Architecture

### Kinematics Layer
- **D1Model** (`model.py`): Pinocchio-based forward kinematics wrapper
- **D1Sim** (`model.py`): MuJoCo simulation and visualization
- **pytorch_kinematics/**: PyTorch-based differentiable FK, Jacobian, and damped least squares IK
- **pyroki_ik/pyroki/**: JAX-based Levenberg-Marquardt IK optimization (pinned to commit `f234516`)

### Calibration Pipeline (`handeye_calib.py`)
```
RealSense Camera → AprilTag Detection → AX=YB Solver (OpenCV)
                                            ↓
                         base_T_cam, cam_T_base transforms
```
Outputs saved to `calib_results/<timestamp>/`

### Grasp Execution Pipeline (`pyroki_ik/`)
```
Grasp Pose (Camera Frame) → cam_T_base Transform → IK Solve → Joint Angles
```

## Key Files

| File | Purpose |
|------|---------|
| `model.py` | D1Model (Pinocchio FK) and D1Sim (MuJoCo) classes |
| `handeye_calib.py` | Interactive hand-eye calibration (requires RealSense + C++ SDK) |
| `handeye_utils.py` | Calibration utilities and transform computation |
| `post_calibration.py` | Error-based filtering of calibration samples |
| `visualization.py` | Real-time robot visualization from C++ SDK joint angles |
| `pyroki_ik/ik2.py` | PyBullet-based IK solver |
| `pyroki_ik/cam2base.py` | Camera-to-base coordinate transformation |

## Robot Configuration

- **URDF:** `assets/urdf/d1_description.urdf`
- **MuJoCo XML:** `assets/mujoco/d1.xml`
- **DOF:** 8 joints (6 arm + 2 gripper)
- **End-effector frame:** `Link6`
- **Joint limits:** approximately ±2.35 radians

## External Integration

- **C++ D1 SDK:** Real robot control via `get_joint_angle` executable (returns 7 joint angles in degrees, comma-separated)
- **RealSense Camera:** RGB-D streams for calibration and grasp detection
- **AprilTag:** Pose estimation with tag_size=0.07614m, tag_id=42

## Coordinate Conventions

- All transforms are 4×4 homogeneous matrices
- Naming convention: `A_T_B` means "transform from B frame to A frame"
- Example: `cam_T_base` transforms points from base frame to camera frame
