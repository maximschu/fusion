import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d

model = YOLO("yolo11n.pt")

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

pipeline_profile = pipeline.get_active_profile()
device = pipeline_profile.get_device()
depth_sensor = device.first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

align = rs.align(rs.stream.color)

profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

#pc = rs.pointcloud()
#decimate = rs.decimation_filter()
#decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
#colorizer = rs.colorizer()

while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    depth_frames = frames.get_depth_frame()
    color_frames = frames.get_color_frame()

    if not depth_frames or not color_frames:
        continue

    depth_image = np.asanyarray(depth_frames.get_data())*depth_scale
    color_image = np.asanyarray(color_frames.get_data())
    
    results = model.track(color_image, persist=True)

    roi = np.zeros((480, 640))
    roi_depth = np.zeros((480, 640))


    if results and len(results[0].boxes):
        for box in results[0].boxes:

            # BOTTOM LEFT, TOP RIGHT
            x1,y1,x2,y2 = map(int, box.xyxy[0])

            # BOX AREA - FIND DEPTHS
            for i in range(x1,x2):
                for j in range(y1, y2):
                    roi[j, i] = depth_frames.get_distance(i, j)
            
            check = np.nonzero(roi)

            # FIND MINIMUM DEPTH / CLOSEST TO CAMERA
            if (check[0].size>0):
                min_depth = np.min(roi[np.nonzero(roi)])
                min_depth_idx = np.where(roi==min_depth)
                
                min_depth_idx_median_y = int(np.median(min_depth_idx[0]))
                min_depth_idx_median_x = int(np.median(min_depth_idx[1]))

            #np.savetxt('test.csv', roi, delimiter=',')

            # MIDPOINT OF BOX
            x_point = ((x2-x1)//2)+x1
            y_point = ((y2-y1)//2)+y1

            depth_obj = depth_frames.get_distance(x_point, y_point)

            # DRAW
            cv2.circle(color_image, (x_point, y_point), radius=2, color=(0, 0, 255), thickness=-1)
            cv2.circle(color_image, (min_depth_idx_median_x, min_depth_idx_median_y), radius=2, color=(0, 0, 255), thickness=-1)
            cv2.rectangle(color_image, (x1, y1), (x2,y2), color=(0,0,255), thickness=2)
            cv2.putText(color_image, f'{depth_obj:.2f}m', (x_point, y_point), color= (0, 255,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
            cv2.putText(color_image, f'{min_depth:.2f}m', (min_depth_idx_median_x, min_depth_idx_median_y), color= (255,0,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)

    cv2.imshow("Depth and YOLOv11", color_image)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
pipeline.stop()
cv2.destroyAllWindows()



