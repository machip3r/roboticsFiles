import numpy as np


def rot(x, y, theta):
    return np.array([[np.cos(theta), -np.sin(theta), x], [np.sin(theta), np.cos(theta), y], [0, 0, 1]])


xFromC = float(input("X desde C: "))
yFromC = float(input("Y desde C: "))

pointC = np.array([[xFromC], [yFromC], [1]])
RbC = rot(0.5, -0.3, -(np.pi / 2))

pointB = RbC.dot(pointC)

RAB = rot(2, 2, (np.pi / 2))
pointWorld = RAB.dot(pointB)

print("\nCoordenadas del punto desde marco de referencia B:")
print(f"[{pointB[0][0]}, {pointB[1][0]}]")

print("\nCoordenadas del punto desde marco de referencia W:")
print(f"[{pointWorld[0][0]}, {pointWorld[1][0]}]")
