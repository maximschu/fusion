import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import csv
from datetime import datetime

# YOLOv11
model = YOLO("yolo11n.pt")

pipeline = rs.pipeline()
config = rs.config()

# depth + colour at 640x480 res
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

csv_file_path = 'camera_depth.csv'
csv_buffer = []
batch_size = 10

with open(csv_file_path, mode='w', newline='') as csv_file:
        fieldnames = ['time', 'label', 'x_point', 'y_point', 'depth']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
            
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # object tracking
            #results = model(color_image)
            results = model.track(color_image, persist=True)

            timestamp = datetime.now()

            # finding depth, confidence score, label + drawing bounding box 
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    confidence_score = box.conf[0].item()
                    label = model.names[int(box.cls[0].item())]

                    x_point = ((x2-x1)//2)+x1
                    y_point = ((y2-y1)//2)+y1

                    depth_obj = depth_frame.get_distance(x_point, y_point)

                    # draw bounding box + labels
                    cv2.circle(color_image, (x_point, y_point), radius=2, color=(0, 0, 255), thickness=-1)
                    cv2.rectangle(color_image, (x1, y1), (x2,y2), color=(0,0,255), thickness=2)
                    cv2.putText(color_image, f'{label}, Score: {confidence_score:.2f}', (x1, y1-20), color= (255, 0,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
                    cv2.putText(color_image, f'{depth_obj:.2f}m', (x_point, y_point), color= (0, 255,0), fontFace= cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)

                    csv_buffer.append({
                        'time': timestamp,
                        'x_point': x_point,
                        'y_point': y_point,
                        'label': label,
                        'depth': depth_obj
                    })

                    print(depth_obj)

            if len(csv_buffer) >= batch_size:
                 writer.writerows(csv_buffer)
                 csv_buffer.clear()

            cv2.imshow("Depth and YOLOv11", color_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

pipeline.stop()
cv2.destroyAllWindows()

            



    
    

    