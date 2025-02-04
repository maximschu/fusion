
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

plt.plot(xccord,yccord,linestyle="",marker=".")






# Create a figure
plt.figure(figsize=(10,8))


# Use extent to map matrix coordinates to world coordinates
plt.imshow(map_matrix, cmap='gray', origin='lower', 
           extent=[x_coords[0], x_coords[-1], y_coords[0], y_coords[-1]])

plt.colorbar(label='Occupancy Probability')
plt.title('SLAM Map from MATLAB')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')

# Show the plot
plt.show()