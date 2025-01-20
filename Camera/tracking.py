import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d
from deep_sort_realtime.deepsort_tracker import DeepSort

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


deepsort = DeepSort(
    max_age=10,        # max number of missed frames before track deleted
    n_init=3,          # min detections before track confirmed
    max_iou_distance=0.9, # max iou distance threshold - lower more strict
    
)

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

    detections = []

    if results and len(results[0].boxes):
        for box in results[0].boxes:

            # BOTTOM LEFT, TOP RIGHT
            x1,y1,x2,y2 = map(int, box.xyxy[0])

            conf = box.conf[0]
            class_id = box.cls[0]
            label = model.names[int(class_id.item())]

            detections.append(([x1, y1, x2, y2], conf, class_id)) ###

        # TRACKING PORTION
        tracks = deepsort.update_tracks(detections, frame = color_image)

        for track in tracks:
            if not track.is_confirmed():
                continue
            track_id = track.track_id

            ltrb = track.to_ltrb()

            cv2.rectangle(color_image, (int(ltrb[0]), int(ltrb[1])), (int(ltrb[2]), int(ltrb[3])), color=(0,0,255), thickness=2)
            cv2.putText(color_image, f'id: {int(track_id)}', (int(ltrb[0]), int(ltrb[1])), color= (0, 255,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
            

    cv2.imshow("YOLOv11 and Tracking", color_image)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
pipeline.stop()
cv2.destroyAllWindows()



