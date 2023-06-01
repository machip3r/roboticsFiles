import numpy as np

pointB = np.array([[1], [1]])
rBA = (np.sqrt(2) / 2) * np.array([[1, -1], [1, 1]])

tBA = np.array([[10], [5]])

pointA = rBA.dot(pointB) + tBA

print(pointA)
