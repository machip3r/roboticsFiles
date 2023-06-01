import math
import numpy as np
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient


def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

r = (0.5 * 0.195)
L = (0.2 * 0.1655)

sim.startSimulation()

x, y, _ = sim.getObjectPosition(robot, -1)
xs = [x]
ys = [y]

fig, ax = plt.subplots()

""" while sim.getSimulationTime() < 15:
    uR, uL = v2u(0.25, 0, r, L)

    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

x, y, _ = sim.getObjectPosition(robot, -1)
xs.append(x)
ys.append(y)

while sim.getSimulationTime() < 30:
    angle = (9 / 5)
    uR, uL = v2u(0, ((angle * math.pi) / 3.5), r, L)

    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

x, y, _ = sim.getObjectPosition(robot, -1)
xs.append(x)
ys.append(y)

while sim.getSimulationTime() < 45:
    uR, uL = v2u(0.25, 0, r, L)

    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

x, y, _ = sim.getObjectPosition(robot, -1)
xs.append(x)
ys.append(y)

while sim.getSimulationTime() < 60:
    angle = (9 / 5)
    uR, uL = v2u(0, ((angle * math.pi) / 3.5), r, L)

    sim.setJointTargetVelocity(motorL, uL)
    sim.setJointTargetVelocity(motorR, uR)

x, y, _ = sim.getObjectPosition(robot, -1)
xs.append(x)
ys.append(y) """

for i in range(1, 10):
    if not (i % 2):
        while sim.getSimulationTime() < (i * 15):
            angle = (9 / 5)
            uR, uL = v2u(0, ((angle * math.pi) / 3.35), r, L)

            sim.setJointTargetVelocity(motorL, uL)
            sim.setJointTargetVelocity(motorR, uR)
    else:
        while sim.getSimulationTime() < (i * 15):
            uR, uL = v2u(0.25, 0, r, L)

            sim.setJointTargetVelocity(motorL, uL)
            sim.setJointTargetVelocity(motorR, uR)

    x, y, _ = sim.getObjectPosition(robot, -1)
    xs.append(x)
    ys.append(y)

sim.stopSimulation()

ax.plot(xs, ys, c="b")
ax.plot(xs[0], ys[0], c="r")
ax.set_title("Trajectory")
plt.show()
