""" 
FUSION CODE 

TRANSFORM RADAR VELOCITY ANGLE

Need to add some progress messages
Need to add timing
very messy debug code
diamond shows the position

more testing required

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
#print(lidardf.columns)
lidardf.columns = lidardf.columns.str.strip()



def add_lidar_to_camera(cameradf, lidardf):
    ###add a timestamp 1 column to convert to datetime
    lidardf['Timestamp1'] = pd.to_datetime(lidardf['Timestamp'])
    

    for _, camera_row in cameradf.iterrows():
        # camera timestamp 't'
        camera_timestamp = pd.to_datetime(camera_row['t'])
        
        # Use the nearest function to find the closest matching lidar timestamp
        closest_lidar_timestamp = min(
            lidardf['Timestamp1'], key=lambda x: abs(x - camera_timestamp)
        )
        
        # Get the corresponding X and Y values from lidardf
        closest_lidar_row = lidardf[lidardf['Timestamp1'] == closest_lidar_timestamp]
        lidar_x = closest_lidar_row['X'].values[0]
        lidar_y = closest_lidar_row['Y'].values[0]
        
        # Add lidar X and Y values to camera X and Y
        cameradf.loc[camera_row.name, 'x'] += lidar_x
        cameradf.loc[camera_row.name, 'y'] += lidar_y

    return cameradf

cameradf = add_lidar_to_camera(cameradf, lidardf)





def add_lidar_to_radar(radardf, lidardf):
   ###add a timestamp 1 column to convert to datetime
    lidardf['Timestamp1'] = pd.to_datetime(lidardf['Timestamp'])

    for _, radar_row in radardf.iterrows():
        # Find the closest lidar timestamp to the radar timestamp 't'
        radar_timestamp = pd.to_datetime(radar_row['t'])
        
        # Use the nearest function to find the closest matching lidar timestamp
        closest_lidar_timestamp = min(
            lidardf['Timestamp1'], key=lambda x: abs(x - radar_timestamp)
        )
        
        # Get the corresponding X and Y values from lidardf
        closest_lidar_row = lidardf[lidardf['Timestamp1'] == closest_lidar_timestamp]
        lidar_x = closest_lidar_row['X'].values[0]
        lidar_y = closest_lidar_row['Y'].values[0]
        #print(f"Radar timestamp: {radar_timestamp}, Closest Lidar timestamp: {closest_lidar_timestamp}")
        #print(f"Radar x, y before: {radar_row['x']}, {radar_row['y']}")
        #print(f"Lidar X, Y: {lidar_x}, {lidar_y}")
        # Add lidar X and Y values to radar X and Y
        radardf.loc[radar_row.name, 'x'] += lidar_x
        radardf.loc[radar_row.name, 'y'] += lidar_y
        #print(f"After update: Radar x, y: {radardf.loc[radar_row.name, 'x']}, {radardf.loc[radar_row.name, 'y']}")
    
    # Add the updated X and Y values to the cameradf
    
    return radardf

radardf = add_lidar_to_radar(radardf, lidardf)


#print(radardf)


lidardf = lidardf.drop(columns=['Timestamp1'])



# Save transformed camera data
csv_path = datafolder + "CameraT.csv"
cameradf.to_csv(csv_path, index=False, header=True)






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

    tthresholds = 0.5
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
            threshold = 1
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
                distthr = 0.3

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

print("HERE")

#print(radardf)
#print(lidardf)
#print(cameradf)
# camera? RADAR Fusion, same logic

for id in cameraset:
    iddf = cameradf.loc[cameradf['ID'].eq(id)]
    tthresholds = 0.5
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
                #print(dist)
                # Threshold in m (RADAR)
                distthr = 0.3
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
    columns={"Timestamp": "t", "ObjectID": "ID",  "PositionX": "x", "PositionY": "y","X":"LOCATION_X","Y":"LOCATION_Y"})
#print(lidardf)
dflist = [lidardf, cameradf, radardf]
concatdf = pd.concat(dflist)
concatdf = concatdf.sort_values(by=['t'])
#print("ALL")
#
# #print(concatdf)

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


            posx=0
            posy=0
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
                posx=obj['LOCATION_X'].mean()
                #print("x",posx)
                posy=obj['LOCATION_Y'].mean()
                #print("y",posy)
                if not (c == 0):
                    vavg = vtotal/c
                    aavg = atotal/c
                else:
                    vavg = 0
                    aavg = 0
                fuseddata.append(
                    [idtotal, ttotal, xavg, yavg, vavg, aavg, totallabel,posx,posy, 0])
                #print(fuseddata)
        else:
            pass

        tframe = tframe + windowtimens
        framedata = []

fuseddf = pd.DataFrame(
    columns=['ID', 't', 'x', 'y', 'v', 'a', 'label','posx','posy', 'hot'], data=fuseddata)
print("WEEEEEEEE",fuseddf)
fuseddf['posx'] = fuseddf.groupby('ID')['posx'].fillna(method='ffill').fillna(method='bfill')
fuseddf['posy'] = fuseddf.groupby('ID')['posy'].fillna(method='ffill').fillna(method='bfill')

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
print("made figures ")
currenttime = 0
figureno = 0


buffer=0.8  
x_min, x_max = min(xccord) - buffer, max(xccord) + buffer
y_min, y_max = min(yccord) - buffer, max(yccord) + buffer


def averagefuseddf(df, window_size=0.5e9, time_col='t'):
    # Create a new DataFrame to store averaged results
    averaged_data = []
    
    # Get the start and end times
    start_time = df[time_col].min()
    end_time = df[time_col].max()
    
    # Iterate over time windows
    current_time = start_time
    while current_time <= end_time:
        # Select rows within the current time window
        window_df = df[(df[time_col] >= current_time) & (df[time_col] < current_time + window_size)]
        
        if not window_df.empty:
            # Group by ID and compute averages
            grouped = window_df.groupby('ID')
            for id_, group in grouped:
                # Average key columns
                avg_x = group['x'].mean()
                avg_y = group['y'].mean()
                avg_posx = group['posx'].mean()
                avg_posy = group['posy'].mean()
                avg_v = group['v'].mean() if not group['v'].isna().all() else 0  # Handle NaN
                avg_a = group['a'].mean() if not group['a'].isna().all() else 0  # Handle NaN
                # Mode for categorical data like label (most common value)
                avg_label = group['label'].mode().iloc[0] if not group['label'].isna().all() else ''
                # Sum for binary 'hot' (1 if any row has hot=1)
                avg_hot = 1 if group['hot'].sum() > 0 else 0
                
                # Store the averaged row
                averaged_data.append({
                    'ID': id_,
                    't': current_time + window_size / 2,  # Midpoint of the window
                    'x': avg_x,
                    'y': avg_y,
                    'posx': avg_posx,
                    'posy': avg_posy,
                    'v': avg_v,
                    'a': avg_a,
                    'label': avg_label,
                    'hot': avg_hot
                })
        
        current_time += window_size
    
    
    averaged_df = pd.DataFrame(averaged_data)
    return averaged_df


#averaged_fuseddf = averagefuseddf(fuseddf, window_size=windowtimens)
#print("Averaged Fused DataFrame:")
#print(averaged_fuseddf)#

# Update fuseddf to use the averaged version for plotting
#fuseddf = averaged_fuseddf

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
            plt.plot(row['posx'],row['posy'],marker="D",mec='#0000FF')
            buffer = 0.8  # Adjust as needed for padding
            ax = plt.gca()
            

           
            

              # Camera FOV
            
            

           
           


            ax = plt.gca()
            ax.set_clip_on(True)  #
            
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            plt.xlim([x_min,x_max])
            plt.ylim([y_min,y_max])



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
            ax = plt.gca()
            cfov = 69  # degrees
            wedge = Wedge((row['posx'], row['posy']), 4, 90-(cfov/2), 90 +
                          (cfov/2), alpha=0.3, color='#0593D5')
            wedge.set_clip_on(True)              
            ax.add_patch(wedge)
            
            irfov = 40
            irwedge = Wedge((row['posx'], row['posy']), 3, 90-(irfov/2), 90-(irfov/2)+irfov, alpha=0.2, color='y')
            wedge.set_clip_on(True)
            ax.add_patch(irwedge)

            radarfov = 160
            radarwedge = Wedge((row['posx'], row['posy']), 30, 90-(radarfov/2), 90-(radarfov/2)+radarfov, alpha=0.1, color='r')
            wedge.set_clip_on(True)
            ax.add_patch(radarwedge)
            plt.legend([wedge, radarwedge, irwedge],['Camera FOV', 'Radar FOV', 'IR FOV'], loc='best' )
            plt.plot(xccord, yccord, linestyle="",
                     marker=",", mfc="k", mec="k")

            ax = plt.gca()

           
            
            # Markers for 25/02 exp
            plt.plot(0, 0, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 0, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 2.5, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(-1, 2.5, linestyle="",
                   marker="*", mfc="r", mec="r")
            
                   
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
            plt.xlim([x_min,x_max])
            plt.ylim([y_min,y_max])

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

print("metrics")






def jerk(df, time_col='t', x_col='x', y_col='y'):
    smoothness_scores = []
    for _, group in df.groupby('ID'):
        if len(group) > 3:  # min3 points
            sorted_group = group.sort_values(time_col)
            dt = sorted_group[time_col].diff().dropna() / 1e9  # convert to seconds
            dx = sorted_group[x_col].diff().dropna()
            dy = sorted_group[y_col].diff().dropna()
            vel_x = dx / dt
            vel_y = dy / dt
            acc_x = vel_x.diff().dropna() / dt.iloc[1:].values
            acc_y = vel_y.diff().dropna() / dt.iloc[1:].values
            jerk_x = acc_x.diff().dropna() / dt.iloc[2:].values
            jerk_y = acc_y.diff().dropna() / dt.iloc[2:].values
            jerk_magnitude = np.sqrt(jerk_x**2 + jerk_y**2)#vector sum
            smoothness_scores.append(np.mean(jerk_magnitude))
    return np.mean(smoothness_scores) if smoothness_scores else np.nan##mean of all ID tracks

# Compute smoothness
lidar_smooth = jerk(lidardf)
camera_smooth = jerk(cameradf)
radar_smooth = jerk(radardf)
fused_smooth = jerk(fuseddf)

print(f"LIDAR Jerk: {lidar_smooth:.3f} m/s³, Camera: {camera_smooth:.3f} m/s³, "
      f"Radar: {radar_smooth:.3f} m/s³, Fused: {fused_smooth:.3f} m/s³")   



      


####continuity????
def tracking_continuity(df, time_window=0.5 * pow(10,9)):
    grouped = df.groupby('ID')
    durations = []
    for _, group in grouped:
        sorted_group = group.sort_values('t')
        time_diffs = sorted_group['t'].diff().dropna()
        gaps = time_diffs[time_diffs > time_window].count()
        if gaps == 0:
            duration = sorted_group['t'].max() - sorted_group['t'].min()
        else:
            duration = (sorted_group['t'].max() - sorted_group['t'].min()) / (gaps + 1)
        durations.append(duration / 1e9)  
    return np.max(durations) if durations else np.nan#####currently should this be max or min? Or the fused oness....


# continuity
lidar_cont = tracking_continuity(lidardf)
camera_cont = tracking_continuity(cameradf)
radar_cont = tracking_continuity(radardf)
fused_cont = tracking_continuity(fuseddf)

print(f"LIDAR Continuity: {lidar_cont:.2f} s, Camera: {camera_cont:.2f} s, "
      f"Radar: {radar_cont:.2f} s, Fused: {fused_cont:.2f} s")


'''# Markers for 25/02 exp
            plt.plot(0, 0, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 0, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(1, 2.5, linestyle="",
                     marker="*", mfc="r", mec="r")
            plt.plot(-1, 2.5, linestyle="",
                   marker="*", mfc="r", mec="r")
            
                   
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
                     marker="*", mfc="r", mec="r")'''