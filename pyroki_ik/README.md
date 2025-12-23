# Inverse Kinematics for Robotic Arm Grasping

This codebase implements a complete inverse kinematics (IK) pipeline for solving grasp poses on a robotic arm. It combines grasp detection with differentiable IK optimization to compute joint angles that position the gripper at target grasp configurations.

## Overview

The pipeline transforms detected grasp poses from camera coordinates to robot base coordinates, then solves for joint angles using JAX-based differentiable kinematics optimization.

```
Grasp Pose (Camera Frame)
         ↓
Camera-to-Base Transform
         ↓
Grasp Pose (Base Frame)
         ↓
IK Optimization (PyRoKi)
         ↓
Joint Angles Solution
         ↓
Visualization & Verification
```

## Components

### 1. Grasp Detection
- Grasp poses are detected using AnyGrasp or similar methods
- Poses are expressed as 4×4 transformation matrices: `T_target_in_cam`
- Multiple candidate grasps can be evaluated

### 2. Coordinate Transformation
- Camera-to-arm-base transformation: `T_cam_in_base` (obtained via hand-eye calibration)
- Transform grasp poses to base frame: `T_target_in_base = T_cam_in_base @ T_target_in_cam`
- File: `cam2base.py`

### 3. Inverse Kinematics Solving
- Uses **PyRoKi**: A modular, differentiable robot kinematics library built on JAX
- Solves IK via Levenberg-Marquardt optimization
- Minimizes weighted least-squares objective combining pose matching and joint constraints
- Files: `pyroki/` module, `01_basic_ik.py`, `ik2.py`

### 4. Visualization
- 3D visualization of grasp poses relative to point clouds
- Multi-angle rendering for validation
- Files: `vis.py`, `vis2.py`

## Prerequisites

### System Requirements
- Python 3.10+
- JAX (with CPU or GPU support)
- PyBullet (for basic physics simulation)
- Open3D (for visualization)

### Installation

1. Clone the repository and install PyRoKi:
```bash
cd pyroki
pip install -e .
```

2. Install additional dependencies:
```bash
pip install numpy scipy pybullet open3d viser yourdfpy jax jaxlib jax-dataclasses jaxlie jaxls
```

3. Verify PyBullet is installed:
```bash
python -c "import pybullet; print(pybullet.__version__)"
```

## Usage

### Step 1: Prepare Input Data

You need the following:
1. **Robot URDF**: Path to the robot description (e.g., `URDF/gripper/d1_description/urdf/d1_description.urdf`)
2. **Camera-to-Base Transform**: `T_cam_in_base` (4×4 matrix) from hand-eye calibration
3. **Grasp Poses**: 4×4 transformation matrices in camera frame

Example input format (`cam2base.py`):
```python
import numpy as np

# Camera-to-base transformation (from hand-eye calibration)
T_cam_in_base = np.array([
    [ 0.01854991, -0.22026411,  0.97526387,  0.36235041],
    [-0.99972266,  0.01006837,  0.02128908, -0.00776866],
    [-0.01450853, -0.97538831, -0.22001626,  0.02456633],
    [ 0.        ,  0.        ,  0.        ,  1.        ],
])

# Grasp pose in camera frame (from AnyGrasp or detector)
T_target_in_cam = np.array([
    [ 0.02277477, -0.99893260, -0.04018656,  0.01049047],
    [ 0.49252543,  0.04619145, -0.86907136, -0.06553812],
    [ 0.87000000,  0.00000000,  0.49305171,  0.23500000],
    [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
])

# Transform to base frame
T_target_in_base = T_cam_in_base @ T_target_in_cam
```

### Step 2: Transform Grasp Pose to Base Frame

Run the coordinate transformation:
```bash
python cam2base.py
```

This outputs `T_target_in_base`, the grasp pose in the robot's base coordinate frame.

### Step 3: Solve Inverse Kinematics

The main IK solver uses PyRoKi with Levenberg-Marquardt optimization:

#### Option A: Interactive IK with Visualization (Viser)

```bash
python 01_basic_ik.py
```

This launches an interactive visualizer where you can:
- Drag the target end-effector position/orientation
- Real-time IK solving and visualization
- See joint angles update in real-time
- Requires a web browser (opens at `http://localhost:8080`)

#### Option B: Direct IK Solving (Non-interactive)

Use the PyBullet-based solver:

```bash
python ik2.py
```

This script:
1. Loads the robot URDF
2. Sets target pose `T_target_in_base`
3. Solves IK using PyBullet's built-in solver
4. Applies joint angles to the robot
5. Verifies end-effector position matches target

**Key parameters in `ik2.py`:**
- `T`: Target transformation matrix (modify this with your `T_target_in_base`)
- `end_effector_link_index`: End-effector link (typically 7 for Link6)
- `maxNumIterations`: Maximum solver iterations (default: 100000)
- `residualThreshold`: Convergence tolerance (default: 1e-6)

#### Option C: Using PyRoKi Directly (Recommended)

For more control and advanced optimization, use PyRoKi directly:

```python
import numpy as np
import pyroki as pk
import pyroki.pyroki_snippets as pks
from yourdfpy import URDF
from scipy.spatial.transform import Rotation as R

# Load robot from URDF
urdf = URDF.load("URDF/gripper/d1_description/urdf/d1_description.urdf")
robot = pk.Robot.from_urdf(urdf)

# Target pose (from coordinate transformation)
T_target_in_base = np.array([...])  # 4x4 matrix

# Extract target position and orientation
target_pos = T_target_in_base[:3, 3]
target_orn = R.from_matrix(T_target_in_base[:3, :3]).as_quat()  # [x, y, z, w]

# Solve IK
solution = pks.solve_ik(
    robot=robot,
    target_link_name="Link6",  # End-effector link
    target_position=target_pos,
    target_wxyz=target_orn,
)

# solution is a numpy array of joint angles
print("Joint angles (radians):", solution)
print("Joint angles (degrees):", np.degrees(solution))
```

### Step 4: Verify and Visualize Results

#### Visualize Grasp Poses with Point Cloud

```bash
python vis.py
```

This renders:
- Object point cloud
- Gripper mesh at grasp pose
- Coordinate frame showing target orientation

#### Multi-Angle Visualization

```bash
python vis2.py
```

Generates multiple views (0°, 90°, 180°, 270°) and saves screenshots.

## Understanding the IK Solver

### PyRoKi Optimization Objective

The IK solver minimizes:

```
minimize: ||pos_residual||² + ||ori_residual||² + ||joint_limit_residual||²
           ↑ position weight=50     ↑ orientation weight=10   ↑ weight=100
```

Where:
- **Pose cost**: Difference between actual and target end-effector pose (position + orientation)
- **Limit cost**: Penalizes joint angles outside valid ranges

### Key Code Sections

**Forward Kinematics** (`pyroki/src/pyroki/_robot.py`):
```python
def forward_kinematics(self, cfg):
    """Compute SE(3) pose of each link given joint configuration."""
```

**IK Cost Function** (`pyroki/src/pyroki/costs/_costs.py`):
```python
@Cost.create_factory
def pose_cost(vals, robot, joint_var, target_pose, target_link_index, 
              pos_weight=50.0, ori_weight=10.0):
    """Residual: SE(3) difference between actual and target pose."""
```

**Solver** (`pyroki/examples/pyroki_snippets/_solve_ik.py`):
```python
def solve_ik(robot, target_link_name, target_wxyz, target_position):
    """Solve IK using Levenberg-Marquardt with JAX."""
```

## Project Structure

```
vis_grasp/
├── README.md                          # This file
├── cam2base.py                        # Coordinate transformation (camera → base)
├── ik.py                              # Basic IK with PyBullet
├── ik2.py                             # Advanced IK with PyBullet (recommended)
├── 01_basic_ik.py                     # Interactive IK with Viser visualization
├── vis.py                             # Single-view 3D visualization
├── vis2.py                            # Multi-angle visualization
├── hand_eye_calib.py                  # Hand-eye calibration (reference only)
├── best_anygrasp_pose.npy             # Cached grasp pose
├── URDF/                              # Robot description files
│   ├── gripper/                       # With gripper
│   └── no_grip/                       # Without gripper
├── pyroki/                            # PyRoKi IK library
│   ├── src/pyroki/
│   │   ├── _robot.py                  # Differentiable kinematics
│   │   ├── costs/                     # Cost functions
│   │   │   ├── _costs.py              # Pose, collision, manipulability costs
│   │   │   └── _pose_cost_*           # Analytical vs numerical Jacobians
│   │   ├── collision/                 # Collision detection
│   │   └── viewer/                    # Visualization utilities
│   ├── examples/                      # Example scripts
│   │   ├── 01_basic_ik.py             # Panda arm IK demo
│   │   └── pyroki_snippets/           # Helper functions
│   └── README.md                      # PyRoKi documentation
├── test_data/                         # Test datasets
└── out_filter_grasp_test/             # Grasp results
```

## Workflow Example

Here's a complete example workflow:

```bash
# 1. Transform grasp pose from camera to base frame
python cam2base.py
# Output: T_target_in_base

# 2. Modify ik2.py with your T_target_in_base
# (Replace the T variable with your computed transformation)

# 3. Solve IK
python ik2.py
# Output: Joint angles in radians and degrees

# 4. Visualize the result
python vis.py
# Output: Renders gripper at grasp pose with point cloud
```

## Configuration & Parameters

### Robot Configuration
- **URDF path**: Change in any script to use different robot models
- **End-effector link**: Typically "Link6" or index 7 in PyBullet
- **Joint limits**: Auto-loaded from URDF, can be overridden

### IK Solver Parameters (PyRoKi)

In `pyroki/examples/pyroki_snippets/_solve_ik.py`:
```python
factors = [
    pk.costs.pose_cost_analytic_jac(
        ...,
        pos_weight=50.0,      # Position error weight
        ori_weight=10.0,      # Orientation error weight
    ),
    pk.costs.limit_cost(
        ...,
        weight=100.0,         # Joint limit violation weight
    ),
]
```

Increase `pos_weight` / `ori_weight` for stricter pose matching.
Increase joint limit `weight` to enforce constraints more strictly.

### PyBullet Solver Parameters (ik.py, ik2.py)

```python
joint_angles = p.calculateInverseKinematics(
    robot_id,
    end_effector_link_index,
    target_pos,
    target_orn,
    maxNumIterations=100000,  # More iterations = better convergence
    residualThreshold=1e-6,   # Smaller = stricter tolerance
)
```

## Troubleshooting

### IK Solver Fails to Converge
- Check target pose is reachable (within arm workspace)
- Increase `maxNumIterations`
- Lower `residualThreshold` for looser tolerance
- Visualize target pose to verify correctness

### Joint Angles Out of Bounds
- Increase joint limit penalty weight in PyRoKi
- Verify URDF joint limits match physical constraints
- Check if target pose is outside workspace

### Coordinate Transformation Errors
- Verify `T_cam_in_base` is correctly calibrated
- Check grasp pose convention matches tool frame
- Apply axis alignment if using different conventions

### Visualization Issues
- Ensure point cloud and gripper files exist in expected paths
- Check URDF path is correct and file is readable
- Verify Open3D installation: `python -c "import open3d; print(open3d.__version__)"`

## Advanced Usage

### Adding Collision Avoidance

```python
# In PyRoKi solver
robot_coll = pk.collision.RobotCollision(robot)
factors.append(
    pk.costs.world_collision_cost(
        robot, robot_coll, joint_var, world_geom,
        margin=0.05, weight=100.0
    )
)
```

### Optimizing for Manipulability

```python
factors.append(
    pk.costs.manipulability_cost(
        robot, joint_var, target_link_index,
        weight=10.0
    )
)
```

### Multi-Target IK

```python
# Solve for multiple target poses
targets = [pose1, pose2, pose3]
for target in targets:
    solution = pks.solve_ik(robot, "Link6", target.pos, target.orn)
    # Apply and verify
```

## References

- **PyRoKi Documentation**: https://chungmin99.github.io/pyroki/
- **PyRoKi Paper**: https://arxiv.org/abs/2505.03728
- **PyBullet Docs**: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwsuQT2fQC_5yAo/

## License & Citation

If you use PyRoKi in your research, please cite:

```bibtex
@misc{pyroki2025,
    title={PyRoki: A Modular Toolkit for Robot Kinematic Optimization},
    author={Chung Min Kim and Brent Yi and Hongsuk Choi and Yi Ma and Ken Goldberg and Angjoo Kanazawa},
    year={2025},
    eprint={2505.03728},
    archivePrefix={arXiv},
    primaryClass={cs.RO},
    url={https://arxiv.org/abs/2505.03728},
}
```

## Support

For issues with:
- **PyRoKi**: See https://github.com/chungmin99/pyroki
- **PyBullet**: See official PyBullet documentation
- **This pipeline**: Check troubleshooting section or verify input data format
