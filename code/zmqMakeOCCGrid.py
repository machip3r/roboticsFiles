"""
Occupancy grid creation using a Pioneer pd3x with ultrasonic sensors.

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""

import os
import cv2
import time
import math as m
import numpy as np
import matplotlib.pyplot as plt
from zmqRemoteApi import RemoteAPIClient


def q2R(x, y, z, w):
    # [x, y, z, w] = q

    R = np.zeros((3, 3))

    R[0, 0] = 1 - 2 * (y ** 2 + z ** 2)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x ** 2 + z ** 2)
    R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 / 2 * (x ** 2 + y ** 2)

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

    for i in range(8):
        state, distance, point, detectedObj, _ = sim.readProximitySensor(
            listSensors[i])
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


client = RemoteAPIClient()
sim = client.getObject("sim")

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

sim.startSimulation()

# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0, 16):
    s = sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3, 1)))

carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

width = 100
height = 100
gridSize = 0.1
ttime = 30

if os.path.exists("map.txt"):
    print("Map found. Loading...")
    occgrid = np.loadtxt("map.txt")
    tocc = (1.0 * (occgrid > 0.5))
    occgrid[occgrid > 0.5] = 0
else:
    print("Creating new map")
    occgrid = (0.5 * np.ones((width, height)))
    tocc = np.zeros((width, height))

t = time.time()
initt = t
niter = 0
while (time.time() - t) < ttime:
    carpos = sim.getObjectPosition(robot, -1)

    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw / gridSize)
    yr = 50 - m.floor(yw / gridSize)
    if xr >= width:
        xr = width
    if yr >= height:
        yr = height
    occgrid[(yr - 1), (xr - 1)] = 0

    carrot = sim.getObjectQuaternion(robot, -1)

    uread = []
    ustate = []
    upt = []
    etime = []
    for i in range(0, 16, 2):
        state, distance, point, detectedObj, _ = sim.readProximitySensor(
            usensor[i])

        uread.append(distance)
        upt.append(point)
        ustate.append(state)

        # Transform detection from sensor frame to robot frame
        if state:
            opos = np.array(point).reshape((3, 1))
        else:
            opos = np.array([0, 0, 1]).reshape((3, 1))

        robs = np.matmul(Rs[i], opos) + Vs[i]

        # Transform detection from robot frame to global frame
        R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
        rpos = np.array(carpos).reshape((3, 1))
        pobs = np.matmul(R, robs) + rpos

        # Transform detection from global frame to occupancy grid cells
        xs = pobs[0]
        ys = pobs[1]
        xo = (width // 2) + m.ceil(xs / gridSize)
        yo = (height // 2) - m.floor(ys / gridSize)
        if xo >= width:
            xo = width
        if yo >= height:
            yo = height
        if state:
            tocc[(yo - 1), (xo - 1)] = 1
        occgrid = cv2.line(occgrid, ((xr - 1), (yr - 1)),
                           ((xo - 1), (yo - 1)), (0, 0, 0), 1)

    # Reactive navigation block
    ul = 1
    ur = 1
    lgains = np.linspace(0, -1, (len(upt) // 2))
    rgains = np.linspace(-1, 0, (len(upt) // 2))

    for k in range(len(upt) // 2):
        if ustate[k]:
            ul += lgains[k] * (1.0 - uread[k])
            ur += rgains[k] * (1.0 - uread[k])
    print("lvel {}   rvel {}".format(ul, ur))

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    """ # List of Detections and States of Sensors
    listDetections, listStates = getDetectionsAndStates(usensor)

    # Velocity and Omega
    errp, errh = getErrpAndErrh(xd, yd)
    v = (Kv * errp)
    omega = (Kh * errh)

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
    errf = sim.setJointTargetVelocity(motorL, uL)
    errf = sim.setJointTargetVelocity(motorR, uR) """

    niter += 1

""" print(lgains)
print(rgains)
finalt = time.time()
print("Avg time per iteration ", ((finalt - initt) / niter)) """

sim.setJointTargetVelocity(motorL, 0)
sim.setJointTargetVelocity(motorR, 0)

sim.stopSimulation()

plt.imshow(tocc + occgrid)
plt.show()
np.savetxt("map.txt", (tocc + occgrid))
