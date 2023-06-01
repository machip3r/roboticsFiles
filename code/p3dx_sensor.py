import time
import numpy as np
import math as m
import scipy.interpolate as spi

from zmqRemoteApi import RemoteAPIClient


def v2u(v, omega, r, L):
    uR = (v / r) + L * omega / (2 * r)
    uL = (v / r) - L * omega / (2 * r)

    return uR, uL


def q2R(q):
    [x, y, z, w] = q

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


client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

sensor3 = sim.getObject("/ultrasonicSensor[3]")
sensor4 = sim.getObject("/ultrasonicSensor[4]")

sim.startSimulation()

while sim.getSimulationTime() < 5:
    state3, distance3, point3, detectedObj3, _ = sim.readProximitySensor(
        sensor3)
    sim.addLog(sim.verbosity_scriptinfos,
               '{} {} {}'.format(state3, distance3, point3))
    state4, distance4, point4, detectedObj4, _ = sim.readProximitySensor(
        sensor4)
    sim.addLog(sim.verbosity_scriptinfos,
               '{} {} {}'.format(state4, distance4, point4))

p3 = np.array([0, 0, distance3]).reshape(3, 1)
sim.addLog(sim.verbosity_scriptinfos,
           'position of pioneer3dx sensor 3 {}'.format(p3))
p4 = np.array([0, 0, distance4]).reshape(3, 1)
sim.addLog(sim.verbosity_scriptinfos,
           'position of pioneer3dx sensor 4 {}'.format(p4))

t3 = np.array(sim.getObjectPosition(sensor3, sim.handle_parent)).reshape(3, 1)
sim.addLog(sim.verbosity_scriptinfos,
           'position cylinder from pioneer3dx sensor 3 {}'.format(t3))
t4 = np.array(sim.getObjectPosition(sensor4, sim.handle_parent)).reshape(3, 1)
sim.addLog(sim.verbosity_scriptinfos,
           'position cylinder from pioneer3dx sensor 4 {}'.format(t4))

R3 = q2R(sim.getObjectQuaternion(sensor3, sim.handle_parent))
sim.addLog(sim.verbosity_scriptinfos,
           'qua (R) cylinder from pioneer3dx sensor 3 {}'.format(R3))
R4 = q2R(sim.getObjectQuaternion(sensor4, sim.handle_parent))
sim.addLog(sim.verbosity_scriptinfos,
           'qua (R) cylinder from pioneer3dx sensor 4 {}'.format(R4))

pc3 = R3.dot(p3) + t3
sim.addLog(sim.verbosity_scriptinfos,
           'p wrt cylinder from pioneer3dx sensor 3 {}'.format(pc3))
pc4 = R4.dot(p4) + t4
sim.addLog(sim.verbosity_scriptinfos,
           'p wrt cylinder from pioneer3dx sensor 4 {}'.format(pc4))

sim.pauseSimulation()
time.sleep(1)

sim.stopSimulation()
