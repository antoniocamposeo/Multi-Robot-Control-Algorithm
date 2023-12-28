import numpy as np
from scipy.integrate import odeint
from Robot_1 import Robot_1
import matplotlib.pyplot as plt
import cubicpath


def plotter():
    plt.plot(y[0, 0], y[0, 1], 'bo')
    # plt.plot(y_1[0, 0], y_1[0, 1], 'ro')
    # plt.plot(y_2[0, 0], y_2[0, 1], 'go')
    plt.plot(y[:, 0], y[:, 1], 'b--', lw=2)
    plt.plot(R1.path[0]['x'],R1.path[0]['y'],'g',lw=2)
    plt.plot(y[-1, 0], y[-1, 1], 'bo')
    # plt.plot(y_1[0, 0], y_1[0, 1], 'ro')
    # plt.plot(y_1[:, 0], y_1[:, 1], 'r--', lw=2)
    # plt.plot(y_1[-1, 0], y_1[-1, 1], 'ro')
    # plt.plot(y_2[:, 0], y_2[:, 1], 'g--', lw=2)
    # plt.plot(y_2[-1, 0], y_2[-1, 1], 'go')

    plt.grid()
    plt.axis('equal')
    plt.xlabel(r'$x$')
    plt.ylabel(r'$Y$')
    plt.legend([r'$Leader$', r'$Follower_1$', r'$Follower_2$'])
    # plt.savefig('Test_Leader_Follower.png')
    plt.show()


if __name__ == '__main__':
    # Trajectory Tracking Control
    R1 = Robot_1('Leader', None, [0, 0, 0], None, None, None, None, None)
    t = np.linspace(0, 20, 1000)
    y = R1.compute_control_1_odeint(t,k=2)
    #R1 = Robot('Leader', None, [0, 0, 0], 0.2, 10, 10, 1, 1)
    #R2 = Robot('Follower', R1, [-1, -1, 0], 0.2, 10, 10, 1, 1)
    #R3 = Robot('Follower', R2, [-1, 1, 0], 0.2, 10, 10, 1, 1)

    # t = np.linspace(0, 10, 20000)
    # y = R1.compute_control_odeint(t)
    # y_1 = R2.compute_control_odeint(t)
    # y_2 = R3.compute_control_odeint(t)

    plotter()
