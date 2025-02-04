import sys
sys.path.append("/home/test/myenv/lib/python3.11/site-packages")
import os
import signal
import time
import logging
import serial
import numpy as np
import cv2 as cv
import cmapy
import csv
from datetime import datetime

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame, remap, cv_filter,\
                         cv_render, RollingAverageFilter,\
                         connect_senxor

import math

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

h = 720
w = 1280
cx,cy = w/2, h/2

# intel camera
#cx = 641.0424194335938
fx = 903.9305419921875


###### WRTIE TO TXT FILE ######
csvFilePath = 'ir_bounding.csv'
csvBuffer = []
batchSize = 10

with open(csvFilePath, mode='w', newline='') as csvFile:
    fieldNames = ['t', 'theta']
    writer = csv.DictWriter(csvFile, fieldnames=fieldNames)
    writer.writeheader()

    while True:
        data, header = mi48.read()
        
        if data is None:
            logger.critical('NONE data received instead of GFRA')
            mi48.stop()
            sys.exit(1)

        ###### TIMESTAMPING ######
        #timestamp = datetime.now()
        #unix_timestamp = timestamp.timestamp()
        timestamp = time.time_ns()

        #csvdata = data.reshape((62, 80))
        #np.savetxt("data.csv", csvdata, delimiter=',')

        ###### FRAME EXTRACT ######
        min_temp = dminav(data.min())  
        max_temp = dmaxav(data.max())  
        frame = data_to_frame(data, (80,62), hflip=True)
        frame = np.clip(frame, min_temp, max_temp)

        ###### ROTATE 90 DEGREES ######
        # frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)

        ###### FILTER + CONVERT UINT8 ######
        filt_uint8 = cv_filter(remap(frame), par, use_median=True, use_bilat=True, use_nlm=True)

        ###### TEMPERATURE THRESHOLD ######
        thresholdTemp = 25
        filt_uint8[frame<thresholdTemp] = 0
                            
        if header is not None:
            logger.debug('  '.join([format_header(header),
                                    format_framestats(data)]))
        else:
            logger.debug(format_framestats(data))

        if GUI:
            height = 620
            width = 800
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
            blur = cv.GaussianBlur(filt_uint8, (7, 7), 0) 

            ###### BINARY / BW ######
            ret, thresh = cv.threshold(blur,60,255,cv.THRESH_BINARY)
            
            ###### RESIZE FRAME ######
            if isinstance(resize, tuple) or isinstance(resize, list):
                cvresize =  cv.resize(thresh, dsize=resize, interpolation=interpolation)
            else:
                cvresize =  cv.resize(thresh, dsize=None, fx=resize, fy=resize, interpolation=interpolation)
            
            ###### ALLOW FOR COLOUR ######
            cvresize_bgr = cv.cvtColor(cvresize, cv.COLOR_GRAY2BGR)
        
            ###### CONTOURS ######
            contours, _ = cv.findContours(cvresize,  cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) 
    
            ###### FILTER OUT SMALL CONTOURS ######
            filtered_contour = []
            for contour in contours:
                #print(cv.contourArea(contour))
                if cv.contourArea(contour) > 70:
                    filtered_contour.append(contour)
            
            if display:
                #print("No of Contours found = " + str(len(filtered_contour))) 
                
                #np.savetxt("data.csv", thresh, delimiter=',')

                cv.imshow("filt_uint8", filt_uint8)
                cv.imshow("blur", blur)
                cv.imshow("BW", thresh)
                
                cv.drawContours(cvresize_bgr, filtered_contour, -1, (0,0,255), 3) 

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

                    cv.rectangle(cvresize_bgr, (extLeft[0], extTop[1]), (extRight[0],extBot[1]), color=(255,0,0), thickness=2)
                    cv.circle(cvresize_bgr, (centerX, centerY),radius=3, color=(255, 0, 0), thickness=-1)
                    d=1
                    
                    opp=((centroidX-cx)/fx)*d
                    adj=d
                    theta = math.atan2(opp,adj)
                    theta = ((theta/3.14159)*180)+15

                    cv.putText(cvresize_bgr, f'{theta}', (centroidX, centroidY), color= (255, 0,0), fontFace= cv.FONT_HERSHEY_SIMPLEX, fontScale= 1, thickness=2)

                    csvBuffer.append({
                            't': timestamp,
                            'theta': theta,
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
                cv.imshow("OUTPUT", cvresize_bgr)

            #cv_render(filt_uint8, resize=(400,310), colormap='ironbow')
            #cv_render(filt_uint8, resize=(800,620), colormap='rainbow2')
            #cv_render(remap(frame), resize=(400,310), colormap='rainbow2')

            key = cv.waitKey(1)  # & 0xFF
            if key == ord("q"):
                break

    mi48.stop()
    cv.destroyAllWindows()
