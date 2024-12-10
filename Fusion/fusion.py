import pandas as pd
import numpy as np
from collections import defaultdict
from functools import reduce
import operator
import itertools
import networkx as nx
import matplotlib.pyplot as plt


xthetad = -90
ythetad = 0
zthetad = 0

xtheta = np.radians(xthetad)
ytheta = np.radians(ythetad)
ztheta = np.radians(zthetad)

x = 0
y = 0.06
z = -0.08

cameradf = pd.read_csv('newcamera.csv')

lidardf = pd.read_csv('LIDAR.csv')

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

lidarset = list(set(lidardf["ObjectID"].to_list()))
lidarset.sort()

print(lidarset)

cameradf = cameradf.sort_values(by=['t'])
lencameradf = len(cameradf)

# 100ms (in s and ns)
tthresholds = 0.1
tthreshold = tthresholds * pow(10,9)

fusiondict = defaultdict(list)

for id in lidarset:
    iddf = lidardf.loc[lidardf['ObjectID'].eq(id)]


    for index, row in iddf.iterrows():
        t = row["Timestamp"]*pow(10,9)
        
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
                distthr = 0.25

                if dist <= distthr:
                    matchlist.append(camerarow['ID'])

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
    objpair = [-k, mode]

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