import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import pandas as pd
import time
from ultralytics import YOLO

# Recording Time (s)
t_total = 15

# File Path
csv_path = 'newcamera.csv'

def append_row(df, row):
    return pd.concat([
                df, 
                pd.DataFrame([row], columns=row.index)]
           ).reset_index(drop=True)

df = pd.DataFrame(columns=('ID', 'label', 'x', 'y', 'z', 't'))

model = YOLO("yolo11n.pt")
#model.conf = 0.2

# set up realsense stream
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# depth scale
pipeline_profile = pipeline.get_active_profile()
device = pipeline_profile.get_device()
depth_sensor = device.first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# for aligning the depth stream to the colour stream (important for accurate readings)
align = rs.align(rs.stream.color)

t_start = time.time_ns()

while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    depth_frames = frames.get_depth_frame()
    color_frames = frames.get_color_frame()

    if not depth_frames or not color_frames:
        continue

    depth_image = np.asanyarray(depth_frames.get_data())*depth_scale
    color_image = np.asanyarray(color_frames.get_data())

    results = model.track(color_image, persist=True, verbose=False)

    if results and len(results[0].boxes):
        for box in results[0].boxes:
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            confidence_score = box.conf[0].item()
            label = model.names[int(box.cls[0].item())]
            if (label != "person"):
               break

            # None Error? Happens only occasionally on startup
            try:
                track_id = int(box.id.item())
            except:
                track_id = 999

            x_point = ((x2-x1)//2)+x1
            y_point = ((y2-y1)//2)+y1
        
            depth_obj = depth_frames.get_distance(x_point, y_point)
            
            # draw bounding box + labels
            cv2.circle(color_image, (x_point, y_point), radius=2, color=(0, 0, 255), thickness=-1)
            cv2.rectangle(color_image, (x1, y1), (x2,y2), color=(0,0,255), thickness=2)
            cv2.putText(color_image, f'{label}, Score: {confidence_score:.2f}', (x1, y1-20), color= (255, 0,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
            cv2.putText(color_image, f'{depth_obj:.2f}m', (x_point, y_point), color= (0, 255,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)

            #print(confidence_score)


            u, v = x_point, y_point
            
            # instrinsics (focal length of image (fx, fy) + principal point (cx, cy))
            instinsics = depth_frames.profile.as_video_stream_profile().intrinsics
            fx, fy = instinsics.fx, instinsics.fy
            cx, cy = instinsics.ppx, instinsics.ppy
            d = depth_frames.get_distance(u,v)

            x = ((u-cx)/fx)*d
            y = ((v-cy)/fy)*d
            z = d
            t = time.time_ns()

            #print("Depth Intrinsics", instinsics)
            #print(f"3D Frame - Label={label}, x_point={x_point:.3f}, y_point={y_point:.3f}, X={x:.3f}, Y={y:.3f}, Z={d:.3f}")
            if not (x == 0 and y == 0 and z == 0):
                new_row = pd.Series({'ID':track_id, 'label':label, 'x': x, 'y': y, 'z': z, 't': t})
                df = append_row(df, new_row)

            if (t-t_start) >= (t_total*1000000000):
                df = df.sort_values(by=['ID', 't'])
                df.to_csv(csv_path, index=False, header=True)
                exit()

    

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow("Depth and YOLOv11", color_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()


