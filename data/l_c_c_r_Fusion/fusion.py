""" 
FUSION CODE 

TRANSFORM RADAR VELOCITY ANGLE

Need to add some progress messages
Need to add timing

"""

# Imports
import os
import os.path
from os import listdir
from os.path import isfile, join

import math
import operator
from collections import defaultdict
from functools import reduce

import pandas as pd
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from scipy.io import loadmat

import moviepy.video.io.ImageSequenceClip

datafolder = "Data/"
if not os.path.exists(datafolder):
    os.makedirs(datafolder)

# Load data from csv into Dataframe
cameradf = pd.read_csv(datafolder + 'Camera.csv')
lidardf = pd.read_csv(datafolder + 'LIDAR.csv')
radardf = pd.read_csv(datafolder + 'RADAR.csv')
irdf = pd.read_csv(datafolder + 'IR.csv')

# CAMERA TRANSFORM
xthetad = -90
ythetad = 0
zthetad = 0

xtheta = np.radians(xthetad)
ytheta = np.radians(ythetad)
ztheta = np.radians(zthetad)

x = 0
y = 0.06
z = -0.08

cameraarray = cameradf[["x", "y", "z"]].to_numpy()


transmat = np.array([
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1]
])

xrotmat = np.array([
    [1, 0, 0, 0],
    [0, np.cos(xtheta), -1*np.sin(xtheta), 0],
    [0, np.sin(xtheta), np.cos(xtheta), 0],
    [0, 0, 0, 1]
])

yrotmat = np.array([
    [np.cos(ytheta), 0, np.sin(ytheta), 0],
    [0, 1, 0, 0],
    [-np.sin(ytheta), 0, np.cos(ytheta), 0],
    [0, 0, 0, 1]
])

zrotmat = np.array([
    [np.cos(ztheta), -1*np.sin(ztheta), 0, 0],
    [np.sin(ztheta), np.cos(ztheta), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

onescolumn = np.ones([len(cameradf), 1])
cameraarray = np.transpose(np.concatenate((cameraarray, onescolumn), 1))

xyrotmat = np.matmul(xrotmat, yrotmat)
xyzrotmat = np.matmul(xyrotmat, zrotmat)
xyzttfmat = np.matmul(xyzrotmat, transmat)

transformed = np.matmul(xyzttfmat, cameraarray)

product = np.delete(np.transpose(transformed), -1, 1)

xcol = product[:, 0]
ycol = product[:, 1]
zcol = product[:, 2]

cameradf['x'] = xcol
cameradf['y'] = ycol
cameradf['z'] = zcol

# Save transformed camera data
csv_path = datafolder + "CameraT.csv"
cameradf.to_csv(csv_path, index=False, header=True)

# RADAR TRANSFORM
xthetad = 180
ythetad = 180
zthetad = -90

xtheta = np.radians(xthetad)
ytheta = np.radians(ythetad)
ztheta = np.radians(zthetad)

x = 0
y = 0
z = 0

radardf["z"] = 0

radararray = radardf[["x", "y", "z"]].to_numpy()


transmat = np.array([
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1]
])

xrotmat = np.array([
    [1, 0, 0, 0],
    [0, np.cos(xtheta), -1*np.sin(xtheta), 0],
    [0, np.sin(xtheta), np.cos(xtheta), 0],
    [0, 0, 0, 1]
])

yrotmat = np.array([
    [np.cos(ytheta), 0, np.sin(ytheta), 0],
    [0, 1, 0, 0],
    [-np.sin(ytheta), 0, np.cos(ytheta), 0],
    [0, 0, 0, 1]
])

zrotmat = np.array([
    [np.cos(ztheta), -1*np.sin(ztheta), 0, 0],
    [np.sin(ztheta), np.cos(ztheta), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

onescolumn = np.ones([len(radardf), 1])
radararray = np.transpose(np.concatenate((radararray, onescolumn), 1))

xyrotmat = np.matmul(xrotmat, yrotmat)
xyzrotmat = np.matmul(xyrotmat, zrotmat)
xyzttfmat = np.matmul(xyzrotmat, transmat)

transformed = np.matmul(xyzttfmat, radararray)

product = np.delete(np.transpose(transformed), -1, 1)

xcol = product[:, 0]
ycol = product[:, 1]
zcol = product[:, 2]

radardf['x'] = xcol
radardf['y'] = ycol
radardf['z'] = zcol

# IR TRANSFORM

# Clockwise Angle Change
dtheta = 0

irdf['theta'] = irdf['theta'] + dtheta


# Preprocess
radardf['t'] = (radardf['t']) * pow(10, 9)
radardf['ID'] = radardf['ID'] + int(500)

lidardf['ObjectID'] = lidardf['ObjectID'] * -1
lidardf["Timestamp"] = lidardf["Timestamp"]*pow(10, 9)

# Save transformed radar data
csv_path = datafolder + "RADART1.csv"
radardf.to_csv(csv_path, index=False, header=True)

# Fuse by LIDAR IDs
lidarset = list(set(lidardf["ObjectID"].to_list()))
lidarset.sort()

cameraset = list(set(cameradf["ID"].to_list()))
cameraset.sort()

cameradf = cameradf.sort_values(by=['t'])
lencameradf = len(cameradf)

fusiondict = defaultdict(list)

for id in lidarset:
    iddf = lidardf.loc[lidardf['ObjectID'].eq(id)]

    tthresholds = 1
    tthreshold = tthresholds * pow(10, 9)

    # LIDAR Camera Fusion

    for index, row in iddf.iterrows():
        t = row["Timestamp"]

        camerat = cameradf["t"].to_list()
        lencamerat = len(camerat)

        matchFound = False
        cameramatcht = 0

        maxidx = len(camerat)
        minidx = 0

        # Binary Search
        while (maxidx - minidx) > 1:
            mididx = int((maxidx+minidx)/2)

            if (camerat[mididx] - tthreshold) <= t <= (camerat[mididx] + tthreshold):
                matchFound = True
                cameramatcht = camerat[mididx]
                break

            elif ((camerat[mididx] - tthreshold) < t):
                minidx = mididx

            elif ((camerat[mididx] + tthreshold) > t):
                maxidx = mididx

        # Check around search result for useful neighbours
        if matchFound:
            threshold = 0.1
            thresholdns = threshold * pow(10, 9)

            left = mididx
            right = left

            while (cameramatcht - camerat[left]) < thresholdns:
                left -= 1
                if left < 1:
                    break

            while (camerat[right] - cameramatcht) < thresholdns:
                right += 1
                if right >= len(camerat) - 1:
                    break

            matchdf = cameradf[left:right]
            matchdf = matchdf.sort_values(by=['ID'])

            matchlist = []

            # Logic for matching IDs (positions within a circle)
            for unused, camerarow in matchdf.iterrows():
                a = np.array((camerarow['x'], camerarow['y']))
                b = np.array((row['PositionX'], row['PositionY']))
                dist = np.linalg.norm(a - b)

                # Threshold in m (LIDAR)
                distthr = 0.2

                if dist <= distthr:
                    matchlist.append(camerarow['ID'])

            if matchlist:
                fusiondict[id].append(matchlist)

objnodes = []
objedges = []

for k, v in fusiondict.items():
    out = reduce(operator.concat, v)
    mode = max(set(out), key=out.count)

    objpair = [k, mode]

    objnodes.extend(objpair)
    objedges.append(objpair)


# camera? RADAR Fusion, same logic

for id in cameraset:
    iddf = cameradf.loc[cameradf['ID'].eq(id)]
    tthresholds = 1
    tthreshold = tthresholds * pow(10, 9)

    for index, row in iddf.iterrows():
        t = row["t"]

        radart = radardf["t"].to_list()
        lenradart = len(radart)

        matchFound = False
        radarmatcht = 0

        maxidx = len(radart)
        minidx = 0

        while (maxidx - minidx) > 1:
            mididx = int((maxidx+minidx)/2)

            if (radart[mididx] - tthreshold) <= t <= (radart[mididx] + tthreshold):
                matchFound = True
                radarmatcht = radart[mididx]
                break

            elif ((radart[mididx] - tthreshold) < t):
                minidx = mididx

            elif ((radart[mididx] + tthreshold) > t):
                maxidx = mididx

        if matchFound:
            threshold = 1
            thresholdns = threshold * pow(10, 9)

            left = mididx
            right = left

            while (radarmatcht - radart[left]) < thresholdns:
                left -= 1
                if left < 1:
                    break

            while (radart[right] - radarmatcht) < thresholdns:
                right += 1
                if right >= len(radart) - 1:
                    break

            matchdf = radardf[left:right]
            matchdf = matchdf.sort_values(by=['ID'])

            matchlist = []

            for unused, radarrow in matchdf.iterrows():
                a = np.array((radarrow['x'], radarrow['y']))
                b = np.array((row['x'], row['y']))
                dist = np.linalg.norm(a - b)

                # Threshold in m (RADAR)
                distthr = 0.1
                if dist <= distthr:
                    matchlist.append(int(radarrow['ID']))

            if matchlist:
                fusiondict[id].append(matchlist)

# Connect IDs in Graph
# Camera Positive IDs (upto 500)
# LIDAR Negative IDs
# RADAR IDs from 500+

for k, v in fusiondict.items():
    out = reduce(operator.concat, v)
    mode = max(set(out), key=out.count)

    objpair = [k, mode]

    objnodes.extend(objpair)
    objedges.append(objpair)

objnodes = list(set(objnodes))


G = nx.Graph()
G.add_nodes_from(objnodes)
G.add_edges_from(objedges)

# list of sets
setlist = list(nx.connected_components(G))
print(setlist)
# Plot ID Graph
# nx.draw(G)
# plt.show()

lidardf = lidardf.rename(
    columns={"Timestamp": "t", "ObjectID": "ID",  "PositionX": "x", "PositionY": "y"})

dflist = [lidardf, cameradf, radardf]
concatdf = pd.concat(dflist)
concatdf = concatdf.sort_values(by=['t'])


starttime = concatdf['t'].iloc[0]
endtime = concatdf['t'].iloc[-1]

windowtime = 0.5
windowtimens = windowtime * pow(10, 9)

duration = endtime - starttime

frames = int(duration//windowtimens)

tframe = starttime

fuseddata = []
framedata = []

# Combine the matched data entries into the fused result

for index, row in concatdf.iterrows():
    framedata.append(row)
    if row['t'] > tframe + windowtimens:
        temp = []
        for entry in framedata:
            for i in range(len(setlist)):
                if int(entry['ID']) in setlist[i]:
                    entry['ID'] = i
                    temp.append(entry)
                    break
        if temp:
            tempdf = pd.DataFrame(temp)
            dflist = [d for _, d in tempdf.groupby(['ID'])]

            xtotal = 0
            ytotal = 0
            vtotal = 0
            atotal = 0
            c = 0
            totallabel = ""
            flag = True
            for obj in dflist:
                ttotal = obj['t'].iloc[0]
                idtotal = obj['ID'].iloc[0]
                for index, row in obj.iterrows():
                    if (type(row['label']) == str):
                        totallabel = row['label']
                    if not (math.isnan(row['Velocity'])):
                        c += 1
                        vtotal += row['Velocity']
                        atotal += row['Angle']

                # Populate t,x,y,v,a,label
                xavg = obj['x'].mean()
                yavg = obj['y'].mean()
                if not (c == 0):
                    vavg = vtotal/c
                    aavg = atotal/c
                else:
                    vavg = 0
                    aavg = 0
                fuseddata.append(
                    [idtotal, ttotal, xavg, yavg, vavg, aavg, totallabel, 0])
        else:
            pass

        tframe = tframe + windowtimens
        framedata = []

fuseddf = pd.DataFrame(
    columns=['ID', 't', 'x', 'y', 'v', 'a', 'label', 'hot'], data=fuseddata)

# IR FUSION
# Add 'hot' tag to objects based on time and angle

tthresholds = 1
tthreshold = tthresholds * pow(10, 9)

for index, row in fuseddf.iterrows():
    t = row["t"]
    irt = irdf["t"].to_list()

    matchFound = False
    irmatcht = 0

    maxidx = len(irt)
    minidx = 0

    if len(irt) == 1:
        matchFound = True

    while (maxidx - minidx) > 1:
        mididx = int((maxidx+minidx)/2)

        if (irt[mididx] - tthreshold) <= t <= (irt[mididx] + tthreshold):
            matchFound = True
            irmatcht = irt[mididx]
            break

        elif ((irt[mididx] - tthreshold) < t):
            minidx = mididx

        elif ((irt[mididx] + tthreshold) > t):
            maxidx = mididx

    if matchFound:
        threshold = 0.1
        thresholdns = threshold * pow(10, 9)

        left = mididx
        right = left

        while (irmatcht - irt[left]) < thresholdns:
            left -= 1
            if left < 1:
                break

        while (irt[right] - irmatcht) < thresholdns:
            right += 1
            if right >= len(irt) - 1:
                break

        matchdf = irdf[left:right]
        # matchdf = matchdf.sort_values(by=['ID'])

        matchlist = []

        for unused, irrow in matchdf.iterrows():
            # Threshold in degrees
            anglethr = 20

            a = math.atan2(row['y'], row['x'])
            b = irrow['theta']

            if (a - anglethr) <= b <= (a + anglethr):
                fuseddf.loc[index, 'hot'] = 1

# Add labels and heat readings onto IDs, for data points where they would be missing
dataframes = [d for _, d in fuseddf.groupby('ID')]

for dataframe in dataframes:
    label = ''
    hot = 0
    tlabel = math.inf
    timehot = math.inf
    for unused, row in dataframe.iterrows():
        if (row['label'] == 'person'):
            if row['t'] < tlabel:
                tlabel = row['t']
                label = 'person'
        if (row['hot'] == 1):
            if row['t'] < timehot:
                timehot = row['t']
                hot = 1
    fuseddf.loc[((fuseddf['ID'] == dataframe['ID'].iloc[0]) &
                 (fuseddf['t'] >= tlabel)), ['label']] = label
    fuseddf.loc[((fuseddf['ID'] == dataframe['ID'].iloc[0])
                 & (fuseddf['t'] >= timehot)), ['hot']] = hot

# Save fused data to csv
fuseddf.to_csv(datafolder + "Fused.csv")

# PLOTTING

# Load the MAT file
data = loadmat(datafolder + 'slamMapData.mat')
map_matrix = data['mapMatrix']
res = data['res'][0, 0]            # since res is scalar, extract it
xLim = data['xLim'][0]            # xLim is a 1x2 array
yLim = data['yLim'][0]            # yLim is a 1x2 array

# Compute coordinate arrays for plotting in world coordinates
# The width and height of the map in cells
height, width = map_matrix.shape

# Generate coordinate arrays based on resolution and world limits
x_coords = np.linspace(xLim[0], xLim[1], width)
y_coords = np.linspace(yLim[0], yLim[1], height)

valuex = (xLim[1]-xLim[0])/width
valuey = (yLim[1]-yLim[0])/height

for i in range(height):
    for j in range(width):
        if map_matrix[i, j] < 0.7:
            map_matrix[i, j] = 0

xccord = []
yccord = []
t = 0

for i in range(height):
    for j in range(width):
        if map_matrix[i, j] > 0.7:
            xccord.append(valuex*j+xLim[0])
            yccord.append(valuey*i+yLim[0])


# Plot and save figures, with logic for Traffic Light System

folder = "Figures/"
if not os.path.exists(folder):
    os.makedirs(folder)

currenttime = 0
figureno = 0

for index, row in fuseddf.iterrows():
    plotted = False

    if currenttime == 0:
        currenttime = row['t']

    while plotted is False:
        if currenttime > row['t']:
            danger = 0
            if (math.sqrt(math.pow(row['x'], 2) + math.pow(row['y'], 2)) < 1.5):
                danger += 1
            if ((row['label'] == 'person') or (row['hot'])):
                danger += 1

            athr = 45
            a = math.atan2(row['y'], row['x'])
            if (row['v'] > 0 and (a - athr <= row['a'] <= a + athr)):
                danger += 1

            mec = 'k'
            if danger == 0:
                mec = 'g'
            elif danger == 1:
                mec = 'y'
            elif danger == 2:
                mec = '#FFA500'  # Orange
            elif danger == 3:
                mec = 'r'

            plt.plot(row['x'], row['y'], marker="x", mec=mec)
            if row['hot'] == 1:
                plt.annotate("ID" + str(row['ID']) + " " +
                             row['label'] + " HOT", (row['x'], row['y']))
            else:
                plt.annotate("ID" + str(row['ID']) + " " +
                             row['label'], (row['x'], row['y']))
            if (row['v'] > 0):
                thresh=-180
                plt.arrow(row['x'], row['y'],-row['v']*math.cos(math.radians(row['a']-thresh)),-row['v']*math.sin(math.radians(row['a']-thresh)))
                #plt.arrow(row['x'], row['y'], (row['x'] + row['v'] *
                 #         math.cos(math.radians(row['a']-thresh))), (row['y'] - row['v']*math.sin(math.radians(row['a']-thresh))))
            plotted = True

        else:
            plt.plot(xccord, yccord, linestyle="",
                     marker=",", mfc="k", mec="k")

            ax = plt.gca()

            # Camera FOV
            cfov = 69  # degrees
            wedge = Wedge((0, 0), 4, 90-(cfov/2), 90 +
                          (cfov/2), alpha=0.3, color='#0593D5')
            ax.add_patch(wedge)

            # Markers for 11/02 Experiments
            plt.plot(0, 0, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(0, 1.2, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(-1.5, 1.2, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 1.2, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(-1.5, -0.3, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 0, linestyle="",
                     marker="*", mfc="r", mec="r")

            ax.set_facecolor("#F8F7F1")  # Background Colour (Grey)
            plt.title(f"Current Time (ns) : {currenttime}")
            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            plt.axis('equal')
            plt.savefig(folder + "figure" + str(figureno))
            plt.clf()
            currenttime += windowtimens
            figureno += 1

# WIP Video Stream

fps = 10

image_files = [f for f in listdir(folder) if isfile(join(folder, f))]
image_files = ['Figures/{0}'.format(element) for element in image_files]

image_files = sorted(image_files, key=lambda x: int(
    x.partition('figure')[2].partition('.')[0]))

clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(
    image_files, fps=fps)
clip.write_videofile('_video.mp4', logger=None, bitrate='8000k')

print("Complete")