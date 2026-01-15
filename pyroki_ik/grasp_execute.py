"""Grasp Execution Module

This module provides functionality to execute a grasp by:
1. Transforming object pose from camera frame to robot base frame
2. Solving inverse kinematics for the grasp pose
3. Planning and executing a trajectory to move the arm to the grasp pose
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional, Tuple
import time

import pyroki as pk
import pyroki_snippets as pks
from yourdfpy import URDF


from d1py.interface import D1Arm


def transform_pose_camera_to_base(
    T_object_in_cam: np.ndarray,
    T_cam_in_base: np.ndarray,
    T_align: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Transform object pose from camera frame to robot base frame.
    
    Args:
        T_object_in_cam: 4x4 transformation matrix of object in camera frame
        T_cam_in_base: 4x4 transformation matrix of camera in base frame
        T_align: Optional 4x4 axis alignment transformation (e.g., for AnyGrasp poses)
    
    Returns:
        T_object_in_base: 4x4 transformation matrix of object in base frame
    """
    if T_align is not None:
        # Apply axis alignment if needed (e.g., for AnyGrasp poses)
        T_object_in_base = T_cam_in_base @ (T_object_in_cam @ T_align)
    else:
        T_object_in_base = T_cam_in_base @ T_object_in_cam
    
    return T_object_in_base


def pose_to_position_quaternion(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract position and quaternion (wxyz format) from transformation matrix.
    
    Args:
        T: 4x4 transformation matrix
    
    Returns:
        position: 3D position vector
        wxyz: quaternion in [w, x, y, z] format (for pyroki)
    """
    position = T[:3, 3]
    rotation_matrix = T[:3, :3]
    
    # Convert rotation matrix to quaternion (xyzw format)
    r = R.from_matrix(rotation_matrix)
    quat_xyzw = r.as_quat()  # [x, y, z, w]
    
    # Convert to wxyz format for pyroki
    wxyz = np.roll(quat_xyzw, 1)  # [w, x, y, z]
    
    return position, wxyz


def create_pre_grasp_pose(
    T_grasp: np.ndarray,
    retract_distance: float = 0.05,
    retract_direction: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Create a pre-grasp pose by retracting along the approach direction.
    
    Args:
        T_grasp: 4x4 transformation matrix of grasp pose
        retract_distance: Distance to retract (in meters)
        retract_direction: Direction to retract (default: negative z-axis of grasp frame)
    
    Returns:
        T_pre_grasp: 4x4 transformation matrix of pre-grasp pose
    """
    if retract_direction is None:
        # Default: retract along negative z-axis of grasp frame
        retract_direction = -T_grasp[:3, 2]  # Negative z-axis
        retract_direction = retract_direction / np.linalg.norm(retract_direction)
    
    T_pre_grasp = T_grasp.copy()
    T_pre_grasp[:3, 3] += retract_direction * retract_distance
    
    return T_pre_grasp


def create_pre_grasp_pose_lift_base_z(
    T_grasp_in_base: np.ndarray,
    lift_distance: float = 0.10,
) -> np.ndarray:
    """
    Create a pre-grasp pose by lifting the grasp pose along the base +Z axis.

    This matches the "10cm higher than grasp" behavior (e.g., approach from above),
    independent of the grasp frame orientation.
    """
    T_pre = T_grasp_in_base.copy()
    T_pre[:3, 3] += np.array([0.0, 0.0, lift_distance], dtype=float)
    return T_pre


def _linspace_cfg(start_cfg: np.ndarray, end_cfg: np.ndarray, timesteps: int) -> np.ndarray:
    """Simple joint-space interpolation (timesteps x dof)."""
    start_cfg = np.asarray(start_cfg, dtype=float)
    end_cfg = np.asarray(end_cfg, dtype=float)
    return np.linspace(start_cfg, end_cfg, int(timesteps))


def solve_ik_for_pose(
    robot: pk.Robot,
    target_link_name: str,
    T_target: np.ndarray
) -> np.ndarray:
    """
    Solve inverse kinematics for a target pose.
    
    Args:
        robot: PyRoKi Robot instance
        target_link_name: Name of the target link (e.g., "Link_tcp")
        T_target: 4x4 transformation matrix of target pose in base frame
    
    Returns:
        joint_angles: Joint angles in radians
    """
    position, wxyz = pose_to_position_quaternion(T_target)
    
    solution = pks.solve_ik(
        robot=robot,
        target_link_name=target_link_name,
        target_position=position,
        target_wxyz=wxyz,
    )
    
    return solution


def solve_trajectory(
    robot: pk.Robot,
    robot_coll: Optional[pk.collision.RobotCollision],
    world_coll: list,
    target_link_name: str,
    start_pose: np.ndarray,
    end_pose: np.ndarray,
    timesteps: int = 25,
    dt: float = 0.02
) -> np.ndarray:
    """
    Solve a trajectory from start pose to end pose.
    
    Args:
        robot: PyRoKi Robot instance
        robot_coll: Robot collision model (optional)
        world_coll: List of world collision geometries
        target_link_name: Name of the target link
        start_pose: 4x4 transformation matrix of start pose
        end_pose: 4x4 transformation matrix of end pose
        timesteps: Number of timesteps in trajectory
        dt: Time step size
    
    Returns:
        trajectory: Array of joint angles for each timestep
    """
    start_pos, start_wxyz = pose_to_position_quaternion(start_pose)
    end_pos, end_wxyz = pose_to_position_quaternion(end_pose)
    
    if robot_coll is None:
        # If no collision model provided, create a simple one
        try:
            robot_coll = pk.collision.RobotCollision.from_urdf(robot.urdf)
        except:
            # Fallback: use simple IK without trajectory optimization
            start_cfg = solve_ik_for_pose(robot, target_link_name, start_pose)
            end_cfg = solve_ik_for_pose(robot, target_link_name, end_pose)
            # Simple linear interpolation
            return np.linspace(start_cfg, end_cfg, timesteps)
    
    traj = pks.solve_trajopt(
        robot=robot,
        robot_coll=robot_coll,
        world_coll=world_coll,
        target_link_name=target_link_name,
        start_position=start_pos,
        start_wxyz=start_wxyz,
        end_position=end_pos,
        end_wxyz=end_wxyz,
        timesteps=timesteps,
        dt=dt,
    )
    
    return np.array(traj)


def transform_d1_joint_angles(solution_rad: np.ndarray) -> np.ndarray:
    """
    Transform joint angles from pyroki convention to D1 robot convention.
    
    Based on the code in 01_basic_ik_raw.py, some joint angles need to be negated.
    
    Args:
        solution_rad: Joint angles in radians from pyroki
    
    Returns:
        solution_deg: Joint angles in degrees for D1 robot (7 joints)
    """
    deg_solution = np.rad2deg(solution_rad)
    deg_solution[0] = -deg_solution[0]
    deg_solution[3] = -deg_solution[3]
    deg_solution = deg_solution[:7]  # Take first 7 joints
    deg_solution[6] = 60  # Set gripper to 60 degrees (open position)
    return deg_solution


def execute_trajectory(
    trajectory: np.ndarray,
    robot_interface: Optional[D1Arm] = None,
    delay_per_step: float = 0.1
) -> bool:
    """
    Execute a trajectory by sending joint angles to the robot.
    
    Args:
        trajectory: Array of joint angles (in radians) for each timestep
        robot_interface: D1Arm robot interface instance
        delay_per_step: Delay between trajectory steps (seconds)
    
    Returns:
        success: True if execution completed successfully
    """
    if robot_interface is None:
        robot_interface = D1Arm()
    
    for i, joint_angles_rad in enumerate(trajectory):
        # Transform to D1 robot format
        joint_angles_deg = transform_d1_joint_angles(joint_angles_rad)
        
        # Send to robot
        robot_interface.set_all_joints(joint_angles_deg, mode=1)
        
        if i < len(trajectory) - 1:  # Don't delay after last step
            time.sleep(delay_per_step)
    
    return True


def grasp_execute(
    T_object_in_cam: np.ndarray,
    T_cam_in_base: np.ndarray,
    urdf_path: str,
    target_link_name: str = "Link_tcp",
    T_align: Optional[np.ndarray] = None,
    use_pre_grasp: bool = True,
    pre_grasp_distance: float = 0.05,
    use_trajectory: bool = True,
    timesteps: int = 25,
    dt: float = 0.02,
    robot_coll: Optional[pk.collision.RobotCollision] = None,
    world_coll: Optional[list] = None,
    current_joint_angles: Optional[np.ndarray] = None,
    execute: bool = True,
    robot_interface: Optional[D1Arm] = None
) -> dict:
    """
    Execute a grasp by solving IK and moving the arm to grasp an object.
    
    This function:
    1. Transforms object pose from camera frame to robot base frame
    2. Optionally creates a pre-grasp pose
    3. Solves inverse kinematics for the grasp pose(s)
    4. Optionally plans a trajectory from current pose to grasp pose
    5. Executes the trajectory by moving the arm
    
    Args:
        T_object_in_cam: 4x4 transformation matrix of object in camera frame
        T_cam_in_base: 4x4 transformation matrix of camera in base frame
        urdf_path: Path to the robot URDF file
        target_link_name: Name of the target link (default: "Link_tcp")
        T_align: Optional axis alignment transformation (e.g., for AnyGrasp poses)
        use_pre_grasp: Whether to use a pre-grasp pose before the final grasp
        pre_grasp_distance: Distance to retract for pre-grasp pose (meters)
        use_trajectory: Whether to plan a trajectory (vs. direct IK solution)
        timesteps: Number of timesteps in trajectory (if use_trajectory=True)
        dt: Time step size for trajectory (if use_trajectory=True)
        robot_coll: Robot collision model (optional, will be created if None and use_trajectory=True)
        world_coll: List of world collision geometries (optional)
        current_joint_angles: Current joint angles in radians (for trajectory planning)
        execute: Whether to actually execute the trajectory on the robot
        robot_interface: D1Arm robot interface instance (will be created if None and execute=True)
    
    Returns:
        result: Dictionary containing:
            - T_object_in_base: Object pose in base frame
            - T_pre_grasp: Pre-grasp pose (if use_pre_grasp=True)
            - T_grasp: Final grasp pose
            - grasp_joint_angles: Joint angles for grasp pose (radians)
            - pre_grasp_joint_angles: Joint angles for pre-grasp pose (radians, if use_pre_grasp=True)
            - trajectory: Trajectory array (if use_trajectory=True)
            - success: Whether execution was successful
    """
    result = {
        'success': False,
        'T_object_in_base': None,
        'T_pre_grasp': None,
        'T_grasp': None,
        'grasp_joint_angles': None,
        'pre_grasp_joint_angles': None,
        'trajectory': None,
    }
    
    try:
        # 1. Transform object pose from camera to base frame
        T_object_in_base = transform_pose_camera_to_base(
            T_object_in_cam, T_cam_in_base, T_align
        )
        result['T_object_in_base'] = T_object_in_base
        
        # 2. Load robot model
        urdf = URDF.load(urdf_path)
        robot = pk.Robot.from_urdf(urdf)
        
        # 3. Create pre-grasp pose if requested
        if use_pre_grasp:
            T_pre_grasp = create_pre_grasp_pose(
                T_object_in_base, retract_distance=pre_grasp_distance
            )
            result['T_pre_grasp'] = T_pre_grasp
        else:
            T_pre_grasp = None
        
        # 4. Solve IK for grasp pose
        print("Solving IK for grasp pose...")
        grasp_joint_angles = solve_ik_for_pose(
            robot, target_link_name, T_object_in_base
        )
        result['grasp_joint_angles'] = grasp_joint_angles
        print(f"Grasp joint angles (deg): {np.rad2deg(grasp_joint_angles)}")
        
        # 5. Solve IK for pre-grasp pose if needed
        if use_pre_grasp and T_pre_grasp is not None:
            print("Solving IK for pre-grasp pose...")
            pre_grasp_joint_angles = solve_ik_for_pose(
                robot, target_link_name, T_pre_grasp
            )
            result['pre_grasp_joint_angles'] = pre_grasp_joint_angles
            print(f"Pre-grasp joint angles (deg): {np.rad2deg(pre_grasp_joint_angles)}")
        else:
            pre_grasp_joint_angles = None
        
        # 6. Plan trajectory if requested
        if use_trajectory:
            print("Planning trajectory...")
            
            # Determine start pose
            if current_joint_angles is not None:
                # Use forward kinematics to get current end-effector pose
                # For now, we'll use the pre-grasp pose as start if available
                # or solve IK for a default pose
                if T_pre_grasp is not None:
                    start_pose = T_pre_grasp
                else:
                    # Use a default start pose (you may want to get this from current joint angles)
                    start_pose = T_pre_grasp if T_pre_grasp is not None else T_object_in_base
            else:
                # Use pre-grasp as start if available, otherwise use grasp pose
                start_pose = T_pre_grasp if T_pre_grasp is not None else T_object_in_base
            
            end_pose = T_object_in_base
            
            # Create collision models if needed
            if robot_coll is None:
                try:
                    robot_coll = pk.collision.RobotCollision.from_urdf(urdf)
                except:
                    robot_coll = None
            
            if world_coll is None:
                world_coll = []
            
            # Solve trajectory
            trajectory = solve_trajectory(
                robot=robot,
                robot_coll=robot_coll,
                world_coll=world_coll,
                target_link_name=target_link_name,
                start_pose=start_pose,
                end_pose=end_pose,
                timesteps=timesteps,
                dt=dt
            )
            result['trajectory'] = trajectory
            print(f"Trajectory planned with {len(trajectory)} timesteps")
        
        # 7. Execute trajectory if requested
        if execute:
            if use_trajectory and result['trajectory'] is not None:
                print("Executing trajectory...")
                success = execute_trajectory(
                    result['trajectory'],
                    robot_interface=robot_interface
                )
            else:
                # Execute direct IK solution
                print("Executing direct IK solution...")
                if robot_interface is None:
                    robot_interface = D1Arm()
                
                joint_angles_deg = transform_d1_joint_angles(grasp_joint_angles)
                robot_interface.set_all_joints(joint_angles_deg, mode=1)
                success = True
            
            result['success'] = success
            if success:
                print("Grasp execution completed successfully!")
            else:
                print("Grasp execution failed.")
        else:
            result['success'] = True
            print("Grasp planning completed (execution skipped).")
        
        result['T_grasp'] = T_object_in_base
        
    except Exception as e:
        print(f"Error during grasp execution: {e}")
        import traceback
        traceback.print_exc()
        result['success'] = False
    
    return result


def grasp_execute_from_okgrasp(
    translation: np.ndarray,
    rotation: np.ndarray,
    T_cam_in_base: np.ndarray,
    urdf_path: str,
    target_link_name: str = "Link_tcp",
    T_align: Optional[np.ndarray] = None,
    use_pre_grasp: bool = True,
    pre_grasp_distance: float = 0.05,
    use_trajectory: bool = True,
    timesteps: int = 25,
    dt: float = 0.02,
    robot_coll: Optional[pk.collision.RobotCollision] = None,
    world_coll: Optional[list] = None,
    current_joint_angles: Optional[np.ndarray] = None,
    execute: bool = True,
    robot_interface: Optional[D1Arm] = None,
) -> dict:
    """
    Convenience wrapper to call grasp_execute using an AnyGrasp / okgrasp pose.

    Args:
        translation: 3D position of the grasp in the camera/grasp frame
        rotation: 3x3 rotation matrix of the grasp in the same frame
        T_cam_in_base: 4x4 transformation matrix of camera in base frame
        urdf_path: Path to the robot URDF file
        Other arguments are forwarded to grasp_execute.

    Returns:
        result dictionary from grasp_execute.
    """
    # Build 4x4 homogeneous transform of grasp/object in camera frame
    T_object_in_cam = np.eye(4)
    T_object_in_cam[:3, :3] = rotation
    T_object_in_cam[:3, 3] = translation

    return grasp_execute(
        T_object_in_cam=T_object_in_cam,
        T_cam_in_base=T_cam_in_base,
        urdf_path=urdf_path,
        target_link_name=target_link_name,
        T_align=T_align,
        use_pre_grasp=use_pre_grasp,
        pre_grasp_distance=pre_grasp_distance,
        use_trajectory=use_trajectory,
        timesteps=timesteps,
        dt=dt,
        robot_coll=robot_coll,
        world_coll=world_coll,
        current_joint_angles=current_joint_angles,
        execute=execute,
        robot_interface=robot_interface,
    )


def plan_pregrasp_and_grasp_from_okgrasp(
    translation: np.ndarray,
    rotation: np.ndarray,
    T_cam_in_base: np.ndarray,
    urdf_path: str,
    *,
    target_link_name: str = "Link_tcp",
    T_align: Optional[np.ndarray] = None,
    pregrasp_lift_m: float = 0.10,
    approach_timesteps: int = 25,
    grasp_timesteps: int = 10,
    dt: float = 0.02,
    start_joint_angles_rad: Optional[np.ndarray] = None,
) -> dict:
    """
    Plan a 2-stage pick motion from an okgrasp best grasp:
    1) Move to a pre-grasp pose defined as (+pregrasp_lift_m in base Z).
    2) Move down to the grasp pose.

    Returns both trajectories (radians + D1 degrees) so the caller can serialize
    them and execute elsewhere (e.g., Jetson + D1Py).
    """
    # Build grasp transform in camera frame
    T_grasp_in_cam = np.eye(4)
    T_grasp_in_cam[:3, :3] = rotation
    T_grasp_in_cam[:3, 3] = translation

    # Transform to base frame
    T_grasp_in_base = transform_pose_camera_to_base(T_grasp_in_cam, T_cam_in_base, T_align)
    T_pregrasp_in_base = create_pre_grasp_pose_lift_base_z(T_grasp_in_base, lift_distance=pregrasp_lift_m)

    # Load robot model
    urdf = URDF.load(urdf_path)
    robot = pk.Robot.from_urdf(urdf)

    # Solve IK for both poses
    pregrasp_q = solve_ik_for_pose(robot, target_link_name, T_pregrasp_in_base)
    grasp_q = solve_ik_for_pose(robot, target_link_name, T_grasp_in_base)

    # Plan simple joint-space trajectories (robust fallback; no trajopt dependency)
    if start_joint_angles_rad is None:
        approach_traj = _linspace_cfg(pregrasp_q, pregrasp_q, approach_timesteps)
    else:
        approach_traj = _linspace_cfg(start_joint_angles_rad, pregrasp_q, approach_timesteps)

    grasp_traj = _linspace_cfg(pregrasp_q, grasp_q, grasp_timesteps)

    # Convert to D1 degrees
    approach_deg = [transform_d1_joint_angles(q.copy()).tolist() for q in approach_traj]
    grasp_deg = [transform_d1_joint_angles(q.copy()).tolist() for q in grasp_traj]

    return {
        "success": True,
        "dt": float(dt),
        "T_grasp_in_base": T_grasp_in_base,
        "T_pregrasp_in_base": T_pregrasp_in_base,
        "pregrasp_q_rad": pregrasp_q,
        "grasp_q_rad": grasp_q,
        "approach_traj_rad": approach_traj,
        "grasp_traj_rad": grasp_traj,
        "approach_traj_deg": approach_deg,
        "grasp_traj_deg": grasp_deg,
    }


# Example usage
if __name__ == "__main__":
    # Example: Object pose in camera frame
    T_object_in_cam = np.array([
        [ 0.02277477, -0.99893260, -0.04018656,  0.01049047],
        [ 0.49252543,  0.04619145, -0.86907136, -0.06553812],
        [ 0.87000000,  0.00000000,  0.49305171,  0.23500000],
        [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
    ])
    
    # Camera pose in robot base frame
    T_cam_in_base = np.array([
        [ 0.01854991, -0.22026411,  0.97526387,  0.36235041],
        [-0.99972266,  0.01006837,  0.02128908, -0.00776866],
        [-0.01450853, -0.97538831, -0.22001626,  0.02456633],
        [ 0.        ,  0.        ,  0.        ,  1.        ],
    ])
    
    # Optional: Axis alignment transformation (for AnyGrasp poses)
    T_align = np.array([
        [0,  0,  1,  0],
        [0,  -1,  0,  0],
        [1,  0,  0,  0],
        [0,  0,  0,  1],
    ])
    
    # URDF path (adjust to your system)
    urdf_path = "URDF/gripper/d1_description/urdf/d1_description.urdf"
    
    # Execute grasp
    result = grasp_execute(
        T_object_in_cam=T_object_in_cam,
        T_cam_in_base=T_cam_in_base,
        urdf_path=urdf_path,
        T_align=T_align,
        use_pre_grasp=True,
        use_trajectory=True,
        execute=False  # Set to True to actually move the robot
    )
    
    print("\nGrasp execution result:")
    print(f"Success: {result['success']}")
    if result['grasp_joint_angles'] is not None:
        print(f"Grasp joint angles (deg): {np.rad2deg(result['grasp_joint_angles'])}")

