import numpy as np
import matplotlib.pyplot as plt
import matplotlib

plt.rcParams['figure.figsize'] = [10 * 2 / 2.54, 8 * 2 / 2.54]
matplotlib.rcParams['figure.dpi'] = 160


def from_polar_to_cartesian(p):
    rho = p[0]
    delta = p[1]
    gamma = p[2]
    x = rho * np.cos(delta + np.pi)
    y = rho * np.sin(delta + np.pi)
    theta = delta - gamma
    return x, y, theta


def from_cartesian_to_polar(q):
    x = q[0]
    y = q[1]
    theta = q[2]
    rho = np.sqrt(x ** 2 + y ** 2)
    gamma = np.arctan2(y, x) - theta + np.pi
    delta = gamma + theta
    return rho, delta, gamma


def plot_xy(x, y, path, u1, u2):
    plt.figure(1)
    plt.plot(x[0], y[0], 'y', marker='x', markersize=5)
    plt.plot(x[-1], y[-1], 'r', marker='o', markersize=5)
    plt.plot(x[:], y[:], 'g--',lw = 2)
    plt.plot(path[0]['x'], path[0]['y'],'b--',lw=2)
    plt.grid()
    plt.xlabel(r'$x$')
    plt.ylabel(r'$y$')
    plt.axis('equal')
    plt.legend(['Start Point','Finish Point','Robot','Path to Follow'])
    plt.show()
