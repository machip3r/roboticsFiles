import time
import numpy as np
import math as m
import scipy.interpolate as spi

from zmqRemoteApi import RemoteAPIClient


def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)

    return m.copysign(angmag, angdir)


client = RemoteAPIClient()
sim = client.getObject("sim")

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

sim.startSimulation()

xs = [0, 2, 2, 0]
ys = [-2, -2, 0, 0]

""" ttime = 200
tarr = np.linspace(0, ttime, len(xsNP))

tnew = np.linspace(0, ttime, 100)
xc = spi.splrep(tarr, xs, s=0)
yc = spi.splrep(tarr, ys, s=0) """

""" for (xd, yd) in zip(xc[1], yc[1]): """
for (xd, yd) in zip(xs, ys):
    Kv = 0.3
    Kh = 0.8
    r = (0.5 * 0.195)
    L = 0.311

    errp = 1000

    while errp > 0.1:
        cPosition = sim.getObjectPosition(robot, -1)
        cRotation = sim.getObjectOrientation(robot, -1)

        errp = m.sqrt(((xd - cPosition[0]) ** 2) + ((yd - cPosition[1]) ** 2))
        angd = m.atan2((yd - cPosition[1]), (xd - cPosition[0]))
        errh = angdiff(cRotation[2], angd)

        print("Distance to goal: {}\nHeading error: {}".format(errp, errh))

        v = (Kv * errp)
        """ if v > 1.5:
            v = 1.5 """
        omega = (Kh * errh)

        uR, uL = v2u(v, omega, r, L)
        sim.setJointTargetVelocity(motorL, uL)
        sim.setJointTargetVelocity(motorR, uR)

time.sleep(1)

sim.stopSimulation()
