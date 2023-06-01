import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

ttime = 200

xRandoms = [i for i in range(10)]
np.random.shuffle(xRandoms)
yRandoms = [i for i in range(10)]
np.random.shuffle(yRandoms)

""" xarr = np.array([0, 3, 2, 3])
yarr = np.array([0, 2, 1, 3]) """
xarr = np.array(xRandoms)
yarr = np.array(yRandoms)
tarr = np.linspace(0, ttime, xarr.shape[0])

tnew = np.linspace(0, ttime, 100)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

plt.figure(1)
xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, ".")
plt.title("Path")
plt.show()

plt.figure(2)
xdot = spi.splev(tnew, xc, der=1)
ydot = spi.splev(tnew, yc, der=1)
plt.plot(tnew, xnew, "b", label="x")
plt.plot(tnew, ynew, "r", label="y")
plt.plot(tnew, xdot, "c", label="xdot")
plt.plot(tnew, ydot, "m", label="ydot")
plt.plot(tarr, xarr, ".")
plt.plot(tarr, yarr, ".")
plt.legend()
plt.title("Position and velocity over time")
plt.show()
