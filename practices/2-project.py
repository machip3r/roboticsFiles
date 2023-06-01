import os
import cv2 as cv
import math as m
import numpy as np
import random as rd
import scipy.interpolate as spi
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient


def q2R(x, y, z, w):
    # [x, y, z, w] = q

    R = np.zeros((3, 3))

    R[0, 0] = 1 - 2 * (y**2 + z**2)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x**2 + z**2)
    R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 / 2 * (x**2 + y**2)

    return R


def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


def angdiff(t1, t2):
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)

    return m.copysign(angmag, angdir)


def getDetectionsAndStates(listSensors):
    listDetections = []
    listStates = []

    for i in range(len(listSensors) // 2):
        state, distance, point, detectedObj, _ = sim.readProximitySensor(listSensors[i])
        listDetections.append(np.linalg.norm(point))
        listStates.append(state)

    return listDetections, listStates


def getErrpAndErrh(xd, yd):
    cPosition = sim.getObjectPosition(robot, -1)
    cRotation = sim.getObjectOrientation(robot, -1)

    errp = m.sqrt(((xd - cPosition[0]) ** 2) + ((yd - cPosition[1]) ** 2))
    angd = m.atan2((yd - cPosition[1]), (xd - cPosition[0]))
    errh = angdiff(cRotation[2], angd)

    return errp, errh


def setRandomPositionCuboids(listCuboids, hCuboids, limitX, limitY):
    xPositions = rd.sample(range((limitX * -1), limitX), len(listCuboids))
    yPositions = rd.sample(range((limitY * -1), limitY), len(listCuboids))

    for i in range(len(listCuboids)):
        sim.setObjectPosition(
            listCuboids[i], -1, [xPositions[i], yPositions[i], hCuboids]
        )

    return xPositions, yPositions


def getListPositionObjects(listCuboids):
    xsObjects = []
    ysObjects = []

    for i in range(len(listCuboids)):
        cuboid = sim.getObjectPosition(listCuboids[i], -1)
        xsObjects.append(cuboid[0])
        ysObjects.append(cuboid[1])

    return xsObjects, ysObjects


def getRsAndVs(listSensors):
    rs = []
    vs = []

    for i in range(len(listSensors)):
        q = sim.getObjectQuaternion(listSensors[i], robot)
        rs.append(q2R(q[0], q[1], q[2], q[3]))
        vs.append(np.reshape(sim.getObjectPosition(listSensors[i], robot), (3, 1)))

    return rs, vs


def avoidObstacles(listSensors, xd, yd, Kv=0.3, Kh=0.8):
    # List of Detections and States of Sensors
    listDetections, listStates = getDetectionsAndStates(listSensors)

    # Velocity and Omega
    errp, errh = getErrpAndErrh(xd, yd)
    v = Kv * errp
    omega = Kh * errh

    # Limits of Velocity and Omega
    if v > 0.5:
        v = 0.5
    if omega > 2.5:
        omega = 2.5
    elif omega < -2.5:
        omega = -2.5

    # Conditions for the sensors
    if listStates[3] and listDetections[3] < 1:
        v = 0.05
        omega = -0.5
    if listStates[4] and listDetections[4] < 1:
        v = 0.05
        omega = 0.5
    if listStates[2] and listDetections[2] < 0.7:
        v = 0.03
        omega = -1
    if listStates[5] and listDetections[5] < 0.7:
        v = 0.03
        omega = 1
    if listStates[1] and listDetections[1] < 0.4:
        v = 0
        omega = -2
    if listStates[6] and listDetections[6] < 0.4:
        v = 0
        omega = 2
    if listStates[0] and listDetections[0] < 0.2:
        v = 0
        omega = -2.5
    if listStates[7] and listDetections[7] < 0.2:
        v = 0
        omega = 2.5

    # Velocity and Omega to Robot
    uR, uL = v2u(v, omega, r, L)
    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

    # Get the Next Point to Go with the Interpolator
    xd = spi.splev(sim.getSimulationTime(), xc, der=0)
    yd = spi.splev(sim.getSimulationTime(), yc, der=0)

    return xd, yd


def getRobotPosition():
    carpos = sim.getObjectPosition(robot, -1)
    xw = carpos[0]
    yw = carpos[1]

    return xw, yw


def mapAndTrajectory(occgrid, tocc, Rs, Vs, mapSize=100, gridSize=0.1):
    xw, yw = getRobotPosition()
    row, col = occgrid.shape
    xr = int(col / 2) + m.ceil(xw / gridSize)
    yr = int(row / 2) - m.floor(yw / gridSize)

    if xr >= col:
        print("Adding end columns")
        for i in range(mapSize):
            occgrid = np.insert(occgrid, -1, 0.5, axis=1)
            tocc = np.insert(tocc, -1, 0, axis=1)

    if xr <= 0:
        print("Adding start columns")
        for i in range(mapSize):
            occgrid = np.insert(occgrid, 0, 0.5, axis=1)
            tocc = np.insert(tocc, 0, 0, axis=1)

    if yr >= row:
        print("Adding end rows")
        for i in range(mapSize):
            occgrid = np.insert(occgrid, -1, 0.5, axis=0)
            tocc = np.insert(tocc, -1, 0, axis=0)

    if yr <= 0:
        print("Adding start rows")
        for i in range(mapSize):
            occgrid = np.insert(occgrid, 0, 0.5, axis=0)
            tocc = np.insert(tocc, 0, 0, axis=0)

    xw, yw = getRobotPosition()

    row, col = occgrid.shape

    xr = int(col / 2) + m.ceil(xw / gridSize)
    yr = int(row / 2) - m.floor(yw / gridSize)

    occgrid[yr - 1, xr - 1] = 0

    uread = []
    ustate = []
    upt = []

    for i in range(0, 16):
        state, distance, point, detectedObj, _ = sim.readProximitySensor(listSensors[i])
        uread.append(np.linalg.norm(point))
        upt.append(point)
        ustate.append(state)

        srot = sim.getObjectQuaternion(listSensors[i], -1)
        spos = sim.getObjectPosition(listSensors[i], -1)
        R = q2R(srot[0], srot[1], srot[2], srot[3])
        spos = np.array(spos).reshape((3, 1))

        if state:
            opos = np.array(point).reshape((3, 1))
        else:
            opos = np.array([0, 0, 1]).reshape((3, 1))

        pobs = np.matmul(R, opos) + spos
        xs = pobs[0]
        ys = pobs[1]
        xo = int(col / 2) + m.ceil(xs / gridSize)
        yo = int(row / 2) - m.floor(ys / gridSize)
        if xo >= col:
            xo = col
        if yo >= row:
            yo = row

        occgrid = cv.line(
            occgrid, ((xr - 1), (yr - 1)), ((xo - 1), (yo - 1)), (0, 0, 0), 1
        )

        if state:
            tocc[yo - 1, xo - 1] = 1


""" COPPELIA API """
client = RemoteAPIClient()
sim = client.getObject("sim")

""" MOTORS, ROBOT, SENSORS AND OBJECTS """
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
listSensors = [sim.getObject(f"/ultrasonicSensor[{i}]") for i in range(16)]
listCuboids = [sim.getObject(f"/Cuboid[{i}]") for i in range(4, 16)]

r = 0.5 * 0.195
L = 2 * 0.1655

""" MAP CONSTANTS """
gridSize = 0.1
mapSize = 100
Rs, Vs = getRsAndVs(listSensors)

occgrid = []
tocc = []

""" TEXT MAP CREATION """
if os.path.exists("map.txt"):
    print("Map found. Loading...")
    occgrid = np.loadtxt("map.txt")
    tocc = 1.0 * (occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print("Creating new map")
    occgrid = 0.5 * np.ones((mapSize, mapSize))
    tocc = np.zeros((mapSize, mapSize))

""" POINTS AND TIME """

sim.startSimulation()

# Simulation Time
ttime = 60 * 10

# First points for the graphs
x, y, _ = sim.getObjectPosition(robot, -1)
xsGraph = [x]
ysGraph = [y]

fig, ax = plt.subplots()

# Limits for the Points to Visit
""" minX = -5
minY = -5
maxX = 5
maxY = 5
nPoints = 10

maxXObjects = 6
maxYObjects = 6 """

# Move of the cuboids in random position
xsObjects, ysObjects = getListPositionObjects(listCuboids)

# Generation of Points to Visit
xs = [
    -6.5,
    6.5,
    6.5,
    -6.5,
    -5.5,
    5.5,
    5.5,
    -5.5,
    -4.5,
    4.5,
    4.5,
    -4.5,
    -3.5,
    3.5,
    3.5,
    -3.5,
    -2.5,
    2.5,
    2.5,
    -2.5,
    -1.5,
    1.5,
    1.5,
    -1.5,
]
ys = [
    7,
    7,
    -7,
    -7,
    6,
    6,
    -6,
    -6,
    5,
    5,
    -5,
    -5,
    4,
    4,
    -4,
    -4,
    3,
    3,
    -3,
    -3,
    2,
    2,
    -2,
    -2,
]
""" xs = np.random.randint(minX, maxX, nPoints)
ys = np.random.randint(minY, maxY, nPoints) """
""" xs = [np.random.uniform(minX, maxX) for i in range(5)]
ys = [np.random.uniform(minY, maxY) for i in range(5)] """

""" xs = np.insert(xs, 0, 0)
ys = np.insert(ys, 0, 0) """

""" CONSTANTS """

Kv = 0.3
Kh = 0.8
errp = 1000

""" SPLINE """

tarr = np.linspace(0, ttime, len(xs))

xc = spi.splrep(tarr, xs, s=0)
yc = spi.splrep(tarr, ys, s=0)

cPosition = sim.getObjectPosition(robot, -1)

xd = spi.splev(cPosition[0], xc, der=0)
yd = spi.splev(cPosition[1], yc, der=0)

""" SIMULATION """
while sim.getSimulationTime() < ttime:
    # Making Map and Trajectory
    mapAndTrajectory(occgrid, tocc, Rs, Vs, mapSize, gridSize)
    # Avoid obstacles
    xd, yd = avoidObstacles(listSensors, xd, yd)

    # Adding Coordinates to Graph Trajectory
    x, y, _ = sim.getObjectPosition(robot, -1)
    xsGraph.append(x)
    ysGraph.append(y)

sim.stopSimulation()

""" GRAPHS """
# Robot Trajectory Graph
plt.figure(1)
ax.plot(xsGraph, ysGraph, ".", color="orange")
ax.plot(xsObjects, ysObjects, "x", color="black")
ax.plot(xsGraph[0], ysGraph[0], ".", color="black")
ax.set_title("Real Trajectory")

# Spline Trajectory Graph
plt.figure(2)
tnew = np.linspace(0, ttime, 100)
xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew, color="orange")
plt.plot(xs, ys, ".", color="black")
plt.title("Desired Trajectory")

plt.figure(3)
plt.imshow(tocc + occgrid)
plt.show()

np.savetxt("map.txt", (tocc + occgrid))
