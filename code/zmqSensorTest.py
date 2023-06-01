"""
    Use of proximity sensors in CoppeliaSim (ZeroMQ API)

    Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
    Mobile Robotics course, University of Guanajuato (2023)
"""

import time
import numpy as np
import math as m

from zmqRemoteApi import RemoteAPIClient


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

sensor = sim.getObject("/sensor")

sim.startSimulation()

while sim.getSimulationTime() < 5:
    state, distance, point, detectedObj, _ = sim.readProximitySensor(sensor)
    sim.addLog(sim.verbosity_scriptinfos,
               '{} {} {}'.format(state, distance, point))

p = np.array([0, 0, distance]).reshape(3, 1)
sim.addLog(sim.verbosity_scriptinfos, 'p {}'.format(p))

t = np.array(sim.getObjectPosition(sensor, sim.handle_parent)).reshape(3, 1)
""" sim.addLog(sim.verbosity_scriptinfos, 'pos {}'.format(
    np.array(sim.getObjectPosition(sensor, sim.handle_world)).reshape(3, 1))) """
sim.addLog(sim.verbosity_scriptinfos, 'pos {}'.format(t))

""" sim.addLog(sim.verbosity_scriptinfos, 'rot {}'.format(
    sim.getObjectOrientation(sensor, sim.handle_world)))
sim.addLog(sim.verbosity_scriptinfos, 'rot {}'.format(
    sim.getObjectOrientation(sensor, sim.handle_parent))) """

""" sim.addLog(sim.verbosity_scriptinfos, 'qua {}'.format(
    sim.getObjectQuaternion(sensor, sim.handle_world)))
sim.addLog(sim.verbosity_scriptinfos, 'qua {}'.format(
    sim.getObjectQuaternion(sensor, sim.handle_parent))) """
""" sim.addLog(sim.verbosity_scriptinfos, 'qua (R) {}'.format(
    q2R(sim.getObjectQuaternion(sensor, sim.handle_world)))) """

R = q2R(sim.getObjectQuaternion(sensor, sim.handle_parent))
sim.addLog(sim.verbosity_scriptinfos, 'qua (R) {}'.format(R))

pc = R.dot(p) + t
sim.addLog(sim.verbosity_scriptinfos, 'p wrt cylinder {}'.format(pc))

sim.pauseSimulation()
time.sleep(1)

sim.stopSimulation()
