import pandas as pd
import numpy as np
from collections import defaultdict
from functools import reduce
import operator
import itertools
import networkx as nx
import matplotlib.pyplot as plt
import math

cameradf = pd.read_csv('camera.csv')

lidardf = pd.read_csv('LIDAR.csv')

radardf = pd.read_csv('RADAR.csv')

# CAMERA
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
    [1,0,0,x],
    [0,1,0,y],
    [0,0,1,z],
    [0,0,0,1]
])

xrotmat = np.array([
    [1,0,0,0],
    [0,np.cos(xtheta),-1*np.sin(xtheta),0],
    [0,np.sin(xtheta),np.cos(xtheta),0],
    [0,0,0,1]
])

yrotmat = np.array([
    [np.cos(ytheta),0,np.sin(ytheta),0],
    [0,1,0,0],
    [-np.sin(ytheta),0,np.cos(ytheta),0],
    [0,0,0,1]
])

zrotmat = np.array([
    [np.cos(ztheta),-1*np.sin(ztheta),0,0],
    [np.sin(ztheta),np.cos(ztheta),0,0],
    [0,0,1,0],
    [0,0,0,1]
])

onescolumn = np.ones([len(cameradf),1])
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

print("Camera:")
print(cameradf)

print("LIDAR:")
print(lidardf)

csv_path = "newcameratransformed.csv"
cameradf.to_csv(csv_path, index=False, header=True)

# RADAR
xthetad = 180
ythetad = 0
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
    [1,0,0,x],
    [0,1,0,y],
    [0,0,1,z],
    [0,0,0,1]
])

xrotmat = np.array([
    [1,0,0,0],
    [0,np.cos(xtheta),-1*np.sin(xtheta),0],
    [0,np.sin(xtheta),np.cos(xtheta),0],
    [0,0,0,1]
])

yrotmat = np.array([
    [np.cos(ytheta),0,np.sin(ytheta),0],
    [0,1,0,0],
    [-np.sin(ytheta),0,np.cos(ytheta),0],
    [0,0,0,1]
])

zrotmat = np.array([
    [np.cos(ztheta),-1*np.sin(ztheta),0,0],
    [np.sin(ztheta),np.cos(ztheta),0,0],
    [0,0,1,0],
    [0,0,0,1]
])

onescolumn = np.ones([len(radardf),1])
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

# Preprocess
radardf['t'] = radardf['t'] * pow(10,9)
radardf['ID'] = radardf['ID'] + int(500)
lidardf['ObjectID'] = lidardf['ObjectID'] * -1
lidardf["Timestamp"] = lidardf["Timestamp"]*pow(10,9)

print("radar:")
print(radardf)

print("LIDAR:")
print(lidardf)

csv_path = "radartransformed.csv"
radardf.to_csv(csv_path, index=False, header=True)

lidarset = list(set(lidardf["ObjectID"].to_list()))
lidarset.sort()

print(lidarset)

cameradf = cameradf.sort_values(by=['t'])
lencameradf = len(cameradf)

fusiondict = defaultdict(list)

for id in lidarset:
    iddf = lidardf.loc[lidardf['ObjectID'].eq(id)]

    tthresholds = 1
    tthreshold = tthresholds * pow(10,9)

    for index, row in iddf.iterrows():
        t = row["Timestamp"]
        print("t:" +str(t))
        
        camerat = cameradf["t"].to_list()
        lencamerat = len(camerat)

        #print("-----------------")
        #print(f"Target t: {t:.0f}")
        #print("-----------------")

        matchFound = False
        cameramatcht = 0

        maxidx = len(camerat)
        minidx = 0

        while (maxidx - minidx) > 1:
            mididx = int((maxidx+minidx)/2)

            if (camerat[mididx] - tthreshold) <= t <= (camerat[mididx] + tthreshold):
                #print("MATCH FOUND")
                #print("DELTA T (s) == " + str(abs((t-camerat[mididx])/pow(10,9))))
                matchFound = True
                cameramatcht = camerat[mididx]
                break

            elif ((camerat[mididx] - tthreshold) < t):
                minidx = mididx

            elif ((camerat[mididx] + tthreshold) > t):
                maxidx = mididx
            

        
        if matchFound:
            threshold = 0.1
            thresholdns = threshold * pow(10,9)

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

            #print(matchdf)

            matchlist = []

            for unused, camerarow in matchdf.iterrows():
                a = np.array((camerarow['x'], camerarow['y']))
                b = np.array((row['PositionX'], row['PositionY']))
                dist = np.linalg.norm(a - b)

                # Threshold in m
                distthr = 1

                if dist <= distthr:
                    matchlist.append(camerarow['ID'])

                #print(f"Id: {id}  ---  dist:{dist}")
            
            if matchlist:
                fusiondict[id].append(matchlist)
    
    # Perform same fusion for RADAR

    # 100ms (in s and ns)
    tthresholds = 2
    tthreshold = tthresholds * pow(10,9)

    for index, row in iddf.iterrows():
        t = row["Timestamp"]
        
        radart = radardf["t"].to_list()
        lenradart = len(radart)

        #print("-----------------")
        #print(f"Target t: {t:.0f}")
        #print("-----------------")

        matchFound = False
        radarmatcht = 0

        maxidx = len(radart)
        minidx = 0

        while (maxidx - minidx) > 1:
            mididx = int((maxidx+minidx)/2)

            if (radart[mididx] - tthreshold) <= t <= (radart[mididx] + tthreshold):
                #print("MATCH FOUND")
                #print("DELTA T (s) == " + str(abs((t-radart[mididx])/pow(10,9))))
                matchFound = True
                radarmatcht = radart[mididx]
                break

            elif ((radart[mididx] - tthreshold) < t):
                minidx = mididx

            elif ((radart[mididx] + tthreshold) > t):
                maxidx = mididx
            

        
        if matchFound:
            threshold = 0.1
            thresholdns = threshold * pow(10,9)

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

            #print(matchdf)

            matchlist = []

            for unused, radarrow in matchdf.iterrows():
                a = np.array((radarrow['x'], radarrow['y']))
                b = np.array((row['PositionX'], row['PositionY']))
                dist = np.linalg.norm(a - b)

                # Threshold in m
                distthr = 2

                if dist <= distthr:
                    matchlist.append(int(radarrow['ID']))

                #print(f"Id: {id}  ---  dist:{dist}")
            
            if matchlist:
                fusiondict[id].append(matchlist)

# Lidar: Camera
print(fusiondict)

#objdict = defaultdict(list)

objnodes = []
objedges = []

for k,v in fusiondict.items():
    print(f"Key:    {k}")
    out = reduce(operator.concat, v)
    print(f"Values: {out}")
    mode = max(set(out), key=out.count)
    print(f"Mode:   {mode}")

    # lidar IDs made negative
    objpair = [k, mode]

    objnodes.extend(objpair)
    objedges.append(objpair)

objnodes = list(set(objnodes))

print(f"objnodes: {objnodes}")
print(f"objedges: {objedges}")

G = nx.Graph()
G.add_nodes_from(objnodes)
G.add_edges_from(objedges)

print("Connected:")
# list of sets
setlist = list(nx.connected_components(G))
print(setlist)

#nx.draw(G)
#plt.show()

lidardf = lidardf.rename(columns={"Timestamp": "t", "ObjectID": "ID",  "PositionX": "x", "PositionY": "y"})
print(lidardf)

dflist = [lidardf, cameradf, radardf]
concatdf = pd.concat(dflist)
concatdf = concatdf.sort_values(by=['t'])

print(concatdf)

starttime = concatdf['t'].iloc[0]
endtime = concatdf['t'].iloc[-1]

print(starttime)
print(endtime)

windowtime = 0.5
windowtimens = windowtime * pow(10,9)

duration = endtime - starttime
print(duration)

frames = int(duration//windowtimens)
print(frames)
print("-----")


tframe = starttime

print(setlist)
print(type(setlist))
print(type(setlist[0]))

fuseddata = []
framedata = []

for index, row in concatdf.iterrows():
    framedata.append(row)
    #print(index)
    if row['t'] > tframe + windowtimens:
        #print("inside")
        #print(len(framedata))
        temp = []
        for entry in framedata:
            #print("ITERATE")
            for i in range(len(setlist)):
                #print(int(entry['ID']))
                #print(setlist)
                if int(entry['ID']) in setlist[i]:
                    entry['ID'] = i
                    temp.append(entry)
                    #print("found")
                    break
        if temp:
            print("-------------")
            #print("exists")
            tempdf = pd.DataFrame(temp)
            print(tempdf)
            dflist = [d for _, d in tempdf.groupby(['ID'])]
            print(dflist)
            xtotal = 0
            ytotal = 0
            vtotal = 0
            atotal = 0
            c = 0
            totallabel = ""
            flag = True
            for obj in dflist:
                print("vvvvvvvvvvvv")
                print(obj)
                print(obj['t'])
                print(type(obj['t']))
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
                if not(c==0):
                    vavg = vtotal/c
                    aavg = atotal/c
                else:
                    vavg = 0
                    aavg = 0
                fuseddata.append([idtotal, ttotal, xavg, yavg, vavg, aavg, totallabel])
        else:
            pass

        tframe = tframe + windowtimens
        framedata = []

print(type(fuseddata[0]))
print(fuseddata[0])
fuseddf = pd.DataFrame(columns=['ID','t','x','y','v','a','label'], data=fuseddata)
print(fuseddf)

fuseddf.to_csv("fused.csv")


from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np


# Load the MAT file
data = loadmat('slamMapData.mat')
map_matrix = data['mapMatrix']
res = data['res'][0,0]            # since res is scalar, extract it
xLim = data['xLim'][0]            # xLim is a 1x2 array
yLim = data['yLim'][0]            # yLim is a 1x2 array

# Compute coordinate arrays for plotting in world coordinates
# The width and height of the map in cells
height, width = map_matrix.shape

# Generate coordinate arrays based on resolution and world limits
x_coords = np.linspace(xLim[0], xLim[1], width)
y_coords = np.linspace(yLim[0], yLim[1], height)
valuex=(xLim[1]-xLim[0])/width
print(valuex)
print("xlim",xLim[0])
valuey=(yLim[1]-yLim[0])/height
print(valuey)
print("Ylim:",yLim[0])
for i in range(height):
    for j in range(width):
        if map_matrix[i,j]<0.7:
            map_matrix[i,j]=0


print(map_matrix)
xccord=[]


yccord=[]
t=0

#plt.imshow(map_matrix)
for i in range(height):
    for j in range(width):
        if map_matrix[i,j]>0.7:
            xccord.append(valuex*j+xLim[0])
            yccord.append(valuey*i+yLim[0])
            
print("xvelues:",xccord)
print("ycord:",yccord)

#windowtime
currenttime = 0
folder = "pictures/"

for index, row in fuseddf.iterrows():

    if currenttime == 0:
        currenttime = row['t']
    
    print("##############")
    print(row['x'])
    print(row['y'])
    print(currenttime - row['t'])

    if currenttime > row['t']:
        plt.plot(row['x'], row['y'], marker="x", mec = 'r')
        if type(row['label']) == str:
            plt.annotate(str(row['ID']) + ": " + row['label'], (row['x'], row['y']))
        if (row['v'] > 0):
            plt.arrow(row['x'], row['y'], row['x'] + row['v']*math.cos(row['a']), row['y'] + row['v']*math.sin(row['a']))
        else:
            plt.annotate(row['ID'], (row['x'], row['y']))
    else:
        plt.plot(xccord,yccord,linestyle="",marker=".", mec = 'k')
        plt.savefig(folder + "figure" + str(index))
        plt.clf()
        currenttime += windowtimens
