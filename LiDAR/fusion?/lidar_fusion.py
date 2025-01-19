import pandas as pd
import numpy as np
from collections import defaultdict
from functools import reduce
import operator
import itertools
import networkx as nx
import matplotlib.pyplot as plt

lidardf = pd.read_csv('filtered.csv')

print("LIDAR:")
print(lidardf)


lidardf.dropna(how='all', inplace=True)

print("LIDAR (NaN rows removed):")
print(lidardf)

lidarset = list(set(lidardf["ObjectID"].to_list()))

lidarset.sort()

print(lidarset)


lidardf["Distance"] = np.sqrt(
    (lidardf["RobotX"] - lidardf["PositionX"]) ** 2 
    + (lidardf["RobotY"] - lidardf["PositionY"]) ** 2######may need to change current column headings
    
)

print(lidardf)

lidardf["Colour"] = np.where(
    lidardf["Distance"] < 0.5, 
    "RED", 
    np.where(lidardf["Distance"] < 1.0, "YELLOW", "GREEN")
)

# Check the result
print(lidardf[["Distance", "Colour"]])

print("FULL DATA")
print(lidardf)
lidardf.to_csv("export.csv", index=False)
