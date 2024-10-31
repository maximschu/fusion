import open3d as o3d
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

align = rs.align(rs.stream.color)

intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault) # default rn

def get_rgbd_image_from_frame(frame):
    color_frame = frame.get_color_frame()
    depth_frame = frame.get_depth_frame()

    if not color_frame or not depth_frame:
        print("...")
        return None

    color_image = o3d.geometry.Image(np.array(color_frame.get_data()))
    depth_image = o3d.geometry.Image(np.array(depth_frame.get_data()))

    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, depth_scale=10.0, depth_trunc=5.0, convert_rgb_to_intensity=False) # sets the max reading range

try:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    rgbd_image = get_rgbd_image_from_frame(aligned_frames)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    
    #pcd = pcd.voxel_down_sample(voxel_size=0.01) 
    #pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    output_filename = "open3d_point_cloud.ply"  # .pcd or .xyz
    o3d.io.write_point_cloud(output_filename, pcd)
    o3d.visualization.draw_geometries([pcd], window_name="pointcloud")

finally:
    pipeline.stop()
    print("Pipeline stopped.")
