import time
import math as m
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient


def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


def angdiff(t1, t2):
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)

    return m.copysign(angmag, angdir)


client = RemoteAPIClient()
sim = client.getObject("sim")

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

sim.startSimulation()

xs = [1.5, 1.5, 0, 0]
ys = [0, -1.5, -1.5, 0]

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
        omega = (Kh * errh)

        uR, uL = v2u(v, omega, r, L)
        sim.setJointTargetVelocity(motorL, uL)
        sim.setJointTargetVelocity(motorR, uR)

time.sleep(1)

sim.stopSimulation()

plt.figure(1)
plt.title("Points to visit")
plt.plot(xs, ys, marker=".", color="red")
plt.show()
