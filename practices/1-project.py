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

    for i in range(len(listSensors)):
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


""" COPPELIA API """
client = RemoteAPIClient()
sim = client.getObject("sim")

""" MOTORS, ROBOT, SENSORS AND OBJECTS """
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
listSensors = [sim.getObject(f"/ultrasonicSensor[{i}]") for i in range(8)]
listCuboids = [sim.getObject(f"/Cuboid[{i}]") for i in range(4, 12)]

r = 0.5 * 0.195
L = 2 * 0.1655

sim.startSimulation()

# Simulation Time
ttime = 60 * 4

# First points for the graphs
x, y, _ = sim.getObjectPosition(robot, -1)
xsGraph = [x]
ysGraph = [y]

fig, ax = plt.subplots()

# Limits for the Points to Visit
minX = -5
minY = -5
maxX = 5
maxY = 5
nPoints = 10

maxXObjects = 6
maxYObjects = 6

# Move of the cuboids in random position
xsObjects, ysObjects = setRandomPositionCuboids(
    listCuboids, 7.5, maxXObjects, maxYObjects
)

# Generation of Points to Visit
""" xs = []
ys = [] """
xs = np.random.randint(minX, maxX, nPoints)
ys = np.random.randint(minY, maxY, nPoints)
""" xs = [np.random.uniform(minX, maxX) for i in range(5)]
ys = [np.random.uniform(minY, maxY) for i in range(5)] """

xs = np.insert(xs, 0, 0)
ys = np.insert(ys, 0, 0)

""" CONSTANTS """

""" Kv = 0.3
Kh = 0.8 """
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

plt.show()
