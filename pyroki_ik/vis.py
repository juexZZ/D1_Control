import open3d as o3d
import glob
import os
import numpy as np

def visualize_cloud_geometries(cloud, geometries, translation = None, rotation = None, visualize = True, save_file = None, render_scene = False, render_coord = True, render_gripper = True):
    """
        cloud       : Point cloud of points
        grippers    : list of grippers of form graspnetAPI grasps
        visualise   : To show windows
        save_file   : Visualisation file name
    """
    visualize=False
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    if translation is not None:
        coordinate_frame1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        translation[2] = -translation[2]
        coordinate_frame1.translate(translation)
        coordinate_frame1.rotate(rotation)

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(visible=visualize)
    for geometry in geometries:
        if render_gripper:
            visualizer.add_geometry(geometry)
        if render_coord:
            # center = np.asarray(geometry.get_center())

            local_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

            T_target_in_cam = np.load("best_anygrasp_pose.npy")
            T_target_in_cam[2, :] = -T_target_in_cam[2, :]
            print("Anygrasp Pose in Opend3D:")
            print(T_target_in_cam)

            local_frame.transform(T_target_in_cam)

            # local_frame.translate(center)

            visualizer.add_geometry(local_frame)
    
    if render_scene:
        visualizer.add_geometry(cloud)
    if translation is not None:
        visualizer.add_geometry(coordinate_frame1)
    visualizer.poll_events()
    visualizer.update_renderer()

    if save_file is not None:
        ## Controlling the zoom
        view_control = visualizer.get_view_control()
        if view_control is not None:
            zoom_scale_factor = 1.4  
            view_control.scale(zoom_scale_factor)
        else:
            print("[WARNING] View control not available (likely headless). Skipping zoom adjustment.")
        visualizer.capture_screen_image(save_file, do_render = True)
        print(f"Saved screen shot visualization at {save_file}")

    if visualize:
        visualizer.add_geometry(coordinate_frame)
        visualizer.run()
    else:
        visualizer.destroy_window() 

def load_cloud_and_grippers(saved_dir):
    cloud = o3d.io.read_point_cloud(os.path.join(saved_dir, "cloud_transformed.ply"))

    grippers = []
    for gripper_path in sorted(glob.glob(os.path.join(saved_dir, "gripper_*.ply"))):
        # Automatically determine if point cloud or mesh
        mesh = o3d.io.read_triangle_mesh(gripper_path)
        # if mesh.has_vertices() and mesh.has_triangles():
        grippers.append(mesh)
        # else:
        #     # fallback: Try to read as point cloud
        #     pc = o3d.io.read_point_cloud(gripper_path)
        #     grippers.append(pc)
    return cloud, grippers

# === Example usage ===
saved_dir = "out_filter_grasp_test"
# saved_dir = "out_clouds"
cloud, grippers = load_cloud_and_grippers(saved_dir)
print(cloud)
print(grippers)
# cloud_loaded = o3d.io.read_point_cloud("out_clouds/cloud_transformed.ply")
# gripper = o3d.io.read_triangle_mesh("out_clouds/gripper_12.ply")

# o3d.visualization.draw_geometries([gripper])



# Ensure your config provides a save path
save_path = "poses2.jpg"

scene = True
coord = True
gripper = True

visualize_cloud_geometries(
    cloud,
    grippers,
    visualize=False,  # Avoid pop-up, suitable for headless server
    save_file=save_path,
    render_scene = scene,
    render_coord = coord,
    render_gripper = gripper
)
