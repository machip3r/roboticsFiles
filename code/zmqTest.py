import time
import math

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

for i in range(1, 6):
    if not (i % 2):
        while sim.getSimulationTime() < (i * 10):
            uR, uL = v2u(0, ((2 * math.pi) / 3) * -1, r, L)
            sim.setJointTargetVelocity(motorL, uL)
            sim.setJointTargetVelocity(motorR, uR)
    else:
        while sim.getSimulationTime() < (i * 10):
            uR, uL = v2u(0.3, 0, r, L)

            sim.setJointTargetVelocity(motorL, uL)
            sim.setJointTargetVelocity(motorR, uR)

sim.stopSimulation()
