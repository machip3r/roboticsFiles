import matplotlib.pyplot as plt
import numpy as np

from scipy.integrate import solve_ivp


def msys(t, y):
    m = 1
    k = 0.1
    c = 0.1
    x1dot = y[1]
    """ x2dot = -k*y[0]/m - c*y[1]/m """
    x2dot = ((-(k * y[0]) - (c * y[1])) / m)

    return np.array([x1dot, x2dot])


""" sol1 = solve_ivp(msys, [0, 100], np.array([5, 0]), max_step=0.1) """
sol1 = solve_ivp(msys, [0, 100], np.array([5, 0]))

print(sol1)
print(sol1.status)

fig = plt.figure()
ax = fig.gca()

# ax.plot(time, sol1[2, :])
# ax.plot(time, solT[2, :])
ax.plot(sol1.t, sol1.y[0, :])

plt.show()
