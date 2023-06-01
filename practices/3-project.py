""" MODULES """
import os
import astarmod
import cv2 as cv
import math as m
import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient

""" UTIL FUNCTIONS """


# Function to calculate the velocity for each wheel
def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


# Function to calculate the angle to rotate the robot
def angdiff(t1, t2):
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)

    return m.copysign(angmag, angdir)


# Function to get errp and errh
def getErrpAndErrh(xd, yd):
    cPosition = sim.getObjectPosition(robot, -1)
    cRotation = sim.getObjectOrientation(robot, -1)

    errp = m.sqrt(((xd - cPosition[0]) ** 2) + ((yd - cPosition[1]) ** 2))
    angd = m.atan2((yd - cPosition[1]), (xd - cPosition[0]))
    errh = angdiff(cRotation[2], angd)

    return errp, errh


# Function to get the actual position of the robot
def getRobotPosition():
    cPosition = sim.getObjectPosition(robot, -1)
    xR = cPosition[0]
    yR = cPosition[1]

    return xR, yR


# Function to follow a list of points of a spline curve
def followPath(xd, yd, Kv=0.3, Kh=0.8):
    errp, errh = getErrpAndErrh(xd, yd)
    v = Kv * errp
    omega = Kh * errh

    uR, uL = v2u(v, omega, r, L)
    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

    xd = spi.splev(sim.getSimulationTime(), xc, der=0)
    yd = spi.splev(sim.getSimulationTime(), yc, der=0)

    return xd, yd


# Function to get the saved map from the text file
def getMapFromFile(mapFilename, floorSize, gridSize):
    occgrid = []
    tocc = []

    if os.path.exists(mapFilename):
        print("Map Loaded")
        occgrid = np.loadtxt(mapFilename)
        tocc = 1.0 * (occgrid > 0.5)
        occgrid[occgrid > 0.5] = 0

    allMap = occgrid + tocc

    return np.array(allMap).reshape(
        (int(floorSize / gridSize) + 1, int(floorSize / gridSize) + 1)
    )


# Function to get the A* route to follow and the routeMap
def astarRoute(routeMap, startPoint):
    cfree = False
    while not cfree:
        randomLocation = np.random.randint(0, routeMap.shape[0], (2,))
        vals = routeMap[randomLocation[0], randomLocation[1]]
        if not vals:
            cfree = True

    route = astarmod.astar(
        routeMap,
        (startPoint[0], startPoint[1]),
        (randomLocation[0], randomLocation[1]),
        allow_diagonal_movement=True,
    )

    rows, columns = astarmod.path2cells(route)
    routeMap[rows, columns] = 2

    return route, routeMap


# Function to dilate the map
def dilateMap(originalMap):
    kernel = np.ones((8, 8), np.uint8)

    return cv.dilate(originalMap, kernel, iterations=1)


# Function to convert coordinates of the world to saved map coordinates
def worldToMap(coordinates, center, gridSize):
    return [
        [int(center[0] - i[1] / gridSize), int(center[1] + i[0] / gridSize)]
        for i in coordinates
    ]


# Function to convert coordinates of the saved map to world coordinates
def mapToWorld(coordinates, center, gridSize):
    return [
        [(i[1] - center[1]) * gridSize, (center[0] - i[0]) * gridSize]
        for i in coordinates
    ]


""" COPPELIA API """
client = RemoteAPIClient()
sim = client.getObject("sim")

""" MOTORS, ROBOT, SENSORS AND OBJECTS """
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

""" ROBOT CONSTANTS """
r = 0.5 * 0.195
L = 2 * 0.1655

""" POINTS AND TIME """
# Simulation time
ttime = 30 * 1

# First points for the graphs
xFirst, yFirst = getRobotPosition()
xsGraph = [xFirst]
ysGraph = [yFirst]

""" MAP CONSTANTS """
mapFilename = "test.txt"
floorSize = 15
mapSize = floorSize * 10
gridSize = 0.1
# Center of the saved map matrix
centerMap = ((floorSize / (2 * gridSize)), (floorSize / (2 * gridSize)))

""" MAP AND ROUTES """
allMap = getMapFromFile(mapFilename, floorSize, gridSize)
# Dilate map
dilatedMap = dilateMap(allMap)
# Rotation to map
dilatedMap = np.rot90(dilatedMap)

# Start coordinates of robot converted in coordinates for the map
startCoords = worldToMap([getRobotPosition()], centerMap, gridSize)
# Generation of A* route and the route drawing on the dilated map
mapRoute, dilatedMap = astarRoute(dilatedMap, startCoords[0])

# Show no dilated map and dilated map with trajectory before start simulation
plt.figure(1)
plt.imshow(allMap)
plt.figure(2)
plt.imshow(dilatedMap)
plt.show()

# Convert coordinates of map to coordinates of world
worldRoute = mapToWorld(mapRoute, centerMap, gridSize)
# List of points to visit
xs = np.array([x[0] for x in worldRoute])
ys = np.array([y[1] for y in worldRoute])

""" CONSTANTS """
Kv = 0.3
Kh = 0.8
errp = 1000

""" SPLINE """
tarr = np.linspace(0, ttime, len(xs))

xc = spi.splrep(tarr, xs, s=0)
yc = spi.splrep(tarr, ys, s=0)

xd = spi.splev(xFirst, xc, der=0)
yd = spi.splev(yFirst, yc, der=0)

sim.startSimulation()

""" SIMULATION """
while sim.getSimulationTime() < ttime:
    # Avoid obstacles
    xd, yd = followPath(xd, yd)

    # Adding Coordinates to Graph Trajectory
    xR, yR = getRobotPosition()
    xsGraph.append(xR)
    ysGraph.append(yR)

sim.stopSimulation()

""" GRAPHS """
# Dilated Map with Trajectory
plt.figure(2)
plt.imshow(dilatedMap)

# Robot Trajectory Graph
plt.figure(3)
plt.plot(xsGraph, ysGraph, ".", color="orange")
plt.title("Real Trajectory")

# Spline Trajectory Graph
plt.figure(4)
tnew = np.linspace(0, ttime, 100)
xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew, color="orange")
plt.plot(xs, ys, ".", color="black")
plt.title("Desired Trajectory")

plt.show()
