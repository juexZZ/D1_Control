import open3d as o3d
import glob
import os
import numpy as np

def visualize_cloud_geometries_multiangle(
    cloud,
    geometries,
    translations=None,
    rotations=None,
    angles=[0, 90, 180, 270],
    zoom=1.4,
    save_dir="views",
    visualize=False
):
    """
    cloud       : o3d.geometry.PointCloud
    geometries  : list of o3d geometries (meshes or pointclouds)
    translations: list of 3-tuples or single 3-tuple
    rotations   : list of 3x3 or 4x4 matrices or single matrix
    angles      : list of yaw angles in degrees to rotate around up-axis
    zoom        : zoom factor for capture
    save_dir    : directory to write screenshots
    visualize   : if True, will pop up the final window instead of headless
    """
    os.makedirs(save_dir, exist_ok=True)

    # 创建 visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=visualize, width=800, height=600)
    vis.add_geometry(cloud)
    for g in geometries:
        vis.add_geometry(g)

    # 如果有平移+旋转的坐标系，也一并加进去
    if translations is not None:
        if not isinstance(translations, list):
            translations = [translations] * len(angles)
        if not isinstance(rotations, list):
            rotations = [rotations] * len(angles)
        for t, R in zip(translations, rotations):
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            t = np.array(t); t[2] = -t[2]
            frame.translate(t)
            frame.rotate(R)
            vis.add_geometry(frame)

    # 获取 view control
    vc = vis.get_view_control()
    vc.set_lookat(cloud.get_center())
    vc.set_up([0, -1, 0])    # y-轴朝下，视角绕 z 轴旋转
    vc.set_front([0, 0, -1]) # 初始朝向
    vc.set_zoom(1.0)

    # 渲染一次（初始化）
    vis.poll_events(); vis.update_renderer()

    # 针对每个角度，绕“上”方向旋转，然后截图
    for idx, yaw in enumerate(angles):
        # 重置到初始视角
        vc.set_front([0, 0, -1])
        vc.set_up([0, -1, 0])
        vc.set_zoom(1.0)

        # 将角度转成弧度
        rad = np.deg2rad(yaw)
        # 基于 yaw 绕 z 轴旋转 front/up
        front = np.array([0, 0, -1])
        up    = np.array([0, -1, 0])
        # Rotation around Z:
        Rz = np.array([
            [ np.cos(rad), -np.sin(rad), 0],
            [ np.sin(rad),  np.cos(rad), 0],
            [          0,           0, 1]
        ])
        vc.set_front(Rz.dot(front).tolist())
        vc.set_up   (Rz.dot(up   ).tolist())
        vc.set_zoom(zoom)

        # 渲染并截图
        vis.poll_events(); vis.update_renderer()
        out_path = os.path.join(save_dir, f"view_{yaw:03d}.png")
        vis.capture_screen_image(out_path, do_render=True)
        print(f"Saved: {out_path}")

    # 如果想在窗口里交互，就打开它
    if visualize:
        vis.run()
    vis.destroy_window()

def load_cloud_and_grippers(saved_dir):
    cloud = o3d.io.read_point_cloud(os.path.join(saved_dir, "cloud_transformed.ply"))

    grippers = []
    for gripper_path in sorted(glob.glob(os.path.join(saved_dir, "gripper_*.ply"))):
        # 自动判断是点云还是 mesh
        mesh = o3d.io.read_triangle_mesh(gripper_path)
        # if mesh.has_vertices() and mesh.has_triangles():
        grippers.append(mesh)
        # else:
        #     # fallback: 尝试读取为点云
        #     pc = o3d.io.read_point_cloud(gripper_path)
        #     grippers.append(pc)
    return cloud, grippers

# 加载点云和几何体
saved_dir = "pcds/out_filter_grasp_test"
cloud, grippers = load_cloud_and_grippers(saved_dir)

# 调用新函数，让它从 0°、90°、180°、270° 这几个角度截图
visualize_cloud_geometries_multiangle(
    cloud,
    grippers,
    None,
    None,
    angles=[0, 90, 180, 270],
    zoom=1.4,
    save_dir="poses_multi",
    visualize=False
)
