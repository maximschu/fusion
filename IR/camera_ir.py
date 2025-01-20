import pyrealsense2 as rs
import numpy as np
import pandas as pd
import time
from ultralytics import YOLO
import sys
sys.path.append("/home/test/myenv/lib/python3.11/site-packages")
import os
import signal
import time
import logging
import cv2 as cv
import cmapy
import csv

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame, remap, cv_filter,\
                         cv_render, RollingAverageFilter,\
                         connect_senxor


############################################# CAMERA INITIALISATION #############################################
# Recording Time (s)
t_total = 150

# File Path
csv_path = 'camera.csv'

def append_row(df, row):
    return pd.concat([
                df, 
                pd.DataFrame([row], columns=row.index)]
           ).reset_index(drop=True)

df = pd.DataFrame(columns=('ID', 'label', 'x', 'y', 'z', 't'))

model = YOLO("yolo11n.pt")

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)

# depth scale
pipeline_profile = pipeline.get_active_profile()
device = pipeline_profile.get_device()
depth_sensor = device.first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# for aligning the depth stream to the colour stream (important for accurate readings)
align = rs.align(rs.stream.color)

t_start = time.time_ns()

############################################# IR CAMERA INITIALISATION #############################################
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

global mi48

def signal_handler(sig, frame):
    logger.info("Exiting due to SIGINT or SIGTERM")
    mi48.stop()
    cv.destroyAllWindows()
    logger.info("Done.")
    sys.exit(0)

def get_colormap(colormap='rainbow2', nc=None):
    try:
        cmap = cv.COLORMAP_JET
    except KeyError:
        cmap = cmapy.cmap(colormap)
    if nc is not None:
        if isinstance(cmap, int):
            try:
                cmap = cmapy.cmap(colormap)
            except KeyError:
                return cmap
       
        nmax = 256
        
        ipc = nmax // nc
        delta = nmax % nc
        lut = [int((j // ipc) / (nc-1) * (nmax-1)) for j in range(nmax-delta)]
        lut += [nmax-1,] * delta
        cmap = np.array([cmap[i] for i in lut], dtype='uint8')
    return cmap

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

mi48, connected_port, port_names = connect_senxor()

logger.info('Camera info:')
logger.info(mi48.camera_info)

if len(sys.argv) == 2:
    STREAM_FPS = int(sys.argv[1])
else:
    STREAM_FPS = 15
mi48.set_fps(STREAM_FPS)

mi48.disable_filter(f1=True, f2=True, f3=True)
mi48.set_filter_1(85)
mi48.enable_filter(f1=True, f2=False, f3=False, f3_ks_5=False)
mi48.set_offset_corr(0.0)

mi48.set_sens_factor(100)
mi48.get_sens_factor()

with_header = True
mi48.start(stream=True, with_header=with_header)

GUI = True

par = {'blur_ks':3, 'd':5, 'sigmaColor': 27, 'sigmaSpace': 27}

dminav = RollingAverageFilter(N=10)
dmaxav = RollingAverageFilter(N=10)

csvFilePath = 'ir_bounding.csv'
csvBuffer = []
batchSize = 10


############################################# MAIN CAMERA #############################################

with open(csvFilePath, mode='w', newline='') as csvFile:
    fieldNames = ['Time', 'centerX', 'centerY']
    writer = csv.DictWriter(csvFile, fieldnames=fieldNames)
    writer.writeheader()

    while True:
        data, header = mi48.read()
        
        if data is None:
            logger.critical('NONE data received instead of GFRA')
            mi48.stop()
            sys.exit(1)

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
                cv.circle(color_image, (x_point, y_point), radius=2, color=(0, 255, 0), thickness=-1)
                cv.rectangle(color_image, (x1, y1), (x2,y2), color=(0,255,0), thickness=2)
                # cv.putText(color_image, f'{label}, Score: {confidence_score:.2f}', (x1, y1-20), color= (255, 0,0), fontFace= cv.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
                cv.putText(color_image, f'{depth_obj:.2f}m', (x_point, y_point), color= (0, 255,0), fontFace= cv.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)

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
                print(f"3D Frame - Label={label}, x_point={x_point:.3f}, y_point={y_point:.3f}, X={x:.3f}, Y={y:.3f}, Z={d:.3f}")
                if not (x == 0 and y == 0 and z == 0):
                    new_row = pd.Series({'ID':track_id, 'label':label, 'x': x, 'y': y, 'z': z, 't': t})
                    df = append_row(df, new_row)

                if (t-t_start) >= (t_total*1000000000):
                    df = df.sort_values(by=['ID', 't'])
                    df.to_csv(csv_path, index=False, header=True)
                    exit()
                

############################################# MAIN IR CAMERA #############################################  
        min_temp = dminav(data.min())  
        max_temp = dmaxav(data.max())  
        frame = data_to_frame(data, (80,62), hflip=True)
        frame = np.clip(frame, min_temp, max_temp)

        timestamp = time.time_ns()

        # frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)

        filt_uint8 = cv_filter(remap(frame), par, use_median=True, use_bilat=True, use_nlm=True)

        thresholdTemp = 25
        filt_uint8[frame<thresholdTemp] = 0
                            
        if header is not None:
            logger.debug('  '.join([format_header(header),
                                    format_framestats(data)]))
        #else:
            #logger.debug(format_framestats(data))
            
        if GUI:
            height = 720 ##################
            width = 1280
            resize= (width, height)
            colormap='rainbow2'
            interpolation=cv.INTER_CUBIC
            display=True
            n_colors=None
            cmap = get_colormap(colormap, n_colors)
            
            ####### COLOUR MAPPING #######
            #cvcol = cv.applyColorMap(filt_uint8, cmap)
            #grey = cv.cvtColor(cvcol, cv.COLOR_BGR2GRAY)

            ###### FILTERING / BLURRING ######
            blur = cv.GaussianBlur(filt_uint8, (9, 9), 0) 

            ###### BINARY / BW ######
            ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY)
            
            ###### RESIZE FRAME ######
            if isinstance(resize, tuple) or isinstance(resize, list):
                cvresize =  cv.resize(thresh, dsize=resize, interpolation=interpolation)
            else:
                cvresize =  cv.resize(thresh, dsize=None, fx=resize, fy=resize, interpolation=interpolation)
            
            ###### ALLOW FOR COLOUR ######
            ir_resized = cv.cvtColor(cvresize, cv.COLOR_GRAY2BGR)


            ###### ALIGNING IR AND CAMERA + DISTORTION CORRECTING ######
            y_offset = 20 # (x = 9.4cm y = 3cm)
            x_offset = 100

            padded_width = color_image.shape[1] + x_offset
            padded_height = color_image.shape[0] + y_offset
            padded_frame = np.zeros((padded_height, padded_width, 3), dtype=np.uint8)


            padded_frame[y_offset:ir_resized.shape[0]+y_offset, x_offset:ir_resized.shape[1]+x_offset] = ir_resized

            ir_aligned = padded_frame[0:color_image.shape[0], 0:color_image.shape[1]]

            h, w = ir_aligned.shape[:2]

            f = (w/2)*(np.tan(np.deg2rad(45)/2))
            cx,cy = w/2, h/2
            camera_mtx = np.array([[f,0,cx], [0,f,cy], [0,0,1]])
            dist_coeff = np.array([-0.018, 0, 0, 0])  # k1 = -0.018 (x = 9.4cm y = 3cm)
            
            new_camera_mtx, _ = cv.getOptimalNewCameraMatrix(camera_mtx, dist_coeff, (w, h), 1, (w, h))
            mapx, mapy = cv.initUndistortRectifyMap(camera_mtx, dist_coeff, None, new_camera_mtx, (w, h), 5)

            ir_undistorted = cv.remap(ir_aligned, mapx, mapy, interpolation=cv.INTER_LINEAR)

            ###### CONTOURS ######
            contours, _ = cv.findContours(cvresize,  cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            ir_undistorted = cv.cvtColor(ir_undistorted, cv.COLOR_BGR2GRAY)
            contours_undist, _ = cv.findContours(ir_undistorted,  cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            ir_undistorted = cv.cvtColor(ir_undistorted, cv.COLOR_GRAY2BGR)
    
            ###### FILTER OUT SMALL CONTOURS ######
            filtered_contour = []
            for contour in contours:
                #print(cv.contourArea(contour))
                if cv.contourArea(contour) > 130:
                    filtered_contour.append(contour)

            
            filtered_contour_undist = []
            for contour in contours_undist:
                #print(cv.contourArea(contour))
                if cv.contourArea(contour) > 130:
                    filtered_contour_undist.append(contour)
            
            if display:
                #print("No of Contours found = " + str(len(filtered_contour))) 
                
                #np.savetxt("data.csv", thresh, delimiter=',')
                
                cv.drawContours(ir_resized, filtered_contour, -1, (0,0,255), 3) 
                cv.drawContours(ir_undistorted, filtered_contour_undist, -1, (0,0,255), 3) 

                ###### FIND BOUNDING BOX, CENTER OF BOUNDING BOX, CENTROID #######
                for filtered in filtered_contour:
                    extLeft = tuple(filtered[filtered[:, :, 0].argmin()][0])
                    extRight = tuple(filtered[filtered[:, :, 0].argmax()][0])
                    extTop = tuple(filtered[filtered[:, :, 1].argmin()][0])
                    extBot = tuple(filtered[filtered[:, :, 1].argmax()][0])

                    centerX = ((extRight[0]-extLeft[0])//2)+extLeft[0]
                    centerY = ((extTop[1]-extBot[1])//2)+extBot[1]

                    M = cv.moments(contour)
                    if M['m00'] != 0:
                        centroidX = int(M['m10'] / M['m00'])
                        centroidY = int(M['m01'] / M['m00'])
                    else:
                        centroidX = centerX
                        centroidY = centerY

                    cv.rectangle(ir_resized, (extLeft[0], extTop[1]), (extRight[0],extBot[1]), color=(255,0,0), thickness=2)
                    cv.circle(ir_resized, (centerX, centerY),radius=3, color=(255, 0, 0), thickness=-1)

                
                for filtered in filtered_contour_undist:
                    extLeft_undist = tuple(filtered[filtered[:, :, 0].argmin()][0])
                    extRight_undist = tuple(filtered[filtered[:, :, 0].argmax()][0])
                    extTop_undist = tuple(filtered[filtered[:, :, 1].argmin()][0])
                    extBot_undist = tuple(filtered[filtered[:, :, 1].argmax()][0])

                    centerX_undist = ((extRight[0]-extLeft[0])//2)+extLeft[0]
                    centerY_undist = ((extTop[1]-extBot[1])//2)+extBot[1]

                    M = cv.moments(contour)
                    if M['m00'] != 0:
                        centroidX_undist = int(M['m10'] / M['m00'])
                        centroidY_undist = int(M['m01'] / M['m00'])
                    else:
                        centroidX_undist = centerX_undist
                        centroidY_undist = centerY_undist

                    cv.rectangle(ir_undistorted, (extLeft_undist[0], extTop_undist[1]), (extRight_undist[0],extBot_undist[1]), color=(255,0,0), thickness=2)
                    cv.circle(ir_undistorted, (centroidX_undist, centroidY_undist),radius=3, color=(255, 0, 0), thickness=-1)

                    csvBuffer.append({
                            'Time': timestamp,
                            'centerX': centerX,
                            'centerY': centerY
                        })
                    
                if len(csvBuffer) >= batchSize:
                    writer.writerows(csvBuffer)
                    csvBuffer.clear()

                #cv.circle(cvresize_bgr, extLeft, 8, (0, 0, 255), -1)
                #cv.circle(cvresize_bgr, extRight, 8, (0, 255, 0), -1)
                #cv.circle(cvresize_bgr, extTop, 8, (255, 0, 0), -1)
                #cv.circle(cvresize_bgr, extBot, 8, (255, 255, 0), -1)

                #cv.putText(cvresize_bgr, f'{blob_id}', (centerX, centerY), color= (255, 0,0), fontFace= cv.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)
                
                ###### OUTPUT BW WITH CONTOURS, BOUNDING BOX, CENTERS ######
                
                cv.imshow("IR", ir_resized)
                overlay = cv.addWeighted(ir_undistorted, 0.5, color_image, 0.5, 0)

        cv.imshow("CAMERA", color_image)

        cv.imshow("Overlay", overlay)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

pipeline.stop()
cv.destroyAllWindows()

