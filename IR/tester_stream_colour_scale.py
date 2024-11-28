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

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame, remap, cv_filter,\
                         cv_render, RollingAverageFilter,\
                         connect_senxor

logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

global mi48

def signal_handler(sig, frame):
    """Ensure clean exit in case of SIGINT or SIGTERM"""
    logger.info("Exiting due to SIGINT or SIGTERM")
    mi48.stop()
    cv.destroyAllWindows()
    logger.info("Done.")
    sys.exit(0)

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


def get_colormap(colormap='rainbow2', nc=None):
    """
    Return a 256-color LUT corresponding to `colormap`.

    `colormap` is either from open cv, matplotlib or explicitly defined above.
    If `nc` is not None, return a quantized colormap with `nc` different colors.
    """
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


while True:
    data, header = mi48.read()
    if data is None:
        logger.critical('NONE data received instead of GFRA')
        mi48.stop()
        sys.exit(1)

    csvdata = data.reshape((62, 80))
    np.savetxt("data.csv", csvdata, delimiter=',')


    min_temp = dminav(data.min())  
    max_temp = dmaxav(data.max())  
    frame = data_to_frame(data, (80,62), hflip=False);
    frame = np.clip(frame, min_temp, max_temp)
    filt_uint8 = cv_filter(remap(frame), par, use_median=True,
                           use_bilat=True, use_nlm=False)
    #
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
        cvcol = cv.applyColorMap(filt_uint8, cmap)

        if isinstance(resize, tuple) or isinstance(resize, list):
            cvresize =  cv.resize(cvcol, dsize=resize,
                                interpolation=interpolation)
        else:
            cvresize =  cv.resize(cvcol, dsize=None, fx=resize, fy=resize,
                                interpolation=interpolation)
        
        temp_bar = np.linspace(0,1,height).reshape(-1,1)
        temp_bar = np.uint8(temp_bar*255)

        temp_bar = cv.applyColorMap(temp_bar, cmap)
        temp_bar = cv.resize(temp_bar, (50, height))
        
        intervals = 10

        for i in range(intervals):
            temp = (((max_temp-min_temp)/(intervals-1))*(i))+min_temp
            y = int((height/intervals)*i)
            label = f'{temp:.1f}'
            cv.putText(temp_bar, label,(5,y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        combined_frame = np.hstack((cvresize, temp_bar))
        
        if display:
            cv.imshow("Thermal", combined_frame)

        #cv_render(filt_uint8, resize=(400,310), colormap='ironbow')
        #cv_render(filt_uint8, resize=(800,620), colormap='rainbow2')
        #cv_render(remap(frame), resize=(400,310), colormap='rainbow2')

        key = cv.waitKey(1)  # & 0xFF
        if key == ord("q"):
            break
#    time.sleep(1)

# stop capture and quit
mi48.stop()
cv.destroyAllWindows()
