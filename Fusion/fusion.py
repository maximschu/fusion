import pandas as pd
import numpy as np


xthetad = -90
ythetad = 180
zthetad = 0

xtheta = np.radians(xthetad)
ytheta = np.radians(ythetad)
ztheta = np.radians(zthetad)

x = 0
y = 0.06
z = -0.08

cameradf = pd.read_csv('camera.csv')
print("OLD:")
print(cameradf)

lidardf = pd.read_csv('lidar.csv')

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

print("NEW:")
print(cameradf)

csv_path = "cameratransformed.csv"
cameradf.to_csv(csv_path, index=False, header=True)