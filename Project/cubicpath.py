import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import os
import time
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = [10 * 2 / 2.54, 8 * 2 / 2.54]
matplotlib.rcParams['figure.dpi'] = 160

def compute_cubic_trajectory(q_i, q_f, k, t=None):
    # Compute Derivate
    x_i = q_i[0]
    y_i = q_i[1]
    x_f = q_f[0]
    y_f = q_f[1]
    theta_i = q_i[2]
    theta_f = q_f[2]

    if t is not None:
        s = t / t[-1]
        tau = 1 / t[-1]
    else:
        s = np.linspace(0, 1, 200)
        tau = 1
    b_x = k * np.cos(q_i[2]) + 3 * x_i
    b_y = k * np.sin(q_i[2]) + 3 * y_i
    a_x = k * np.cos(q_f[2]) - 3 * x_f
    a_y = k * np.sin(q_f[2]) - 3 * y_f

    # Cartesian cubic path
    x = x_f * s ** 3 - x_i * (s - 1) ** 3 + a_x * s ** 2 * (s - 1) + b_x * s * (s - 1) ** 2
    y = y_f * s ** 3 - y_i * (s - 1) ** 3 + a_y * s ** 2 * (s - 1) + b_y * s * (s - 1) ** 2

    # Compute first derivate
    xp = 3 * x_f * s ** 2 - 3 * x_i * (s - 1) ** 2 + a_x * (3 * s ** 2 - 2 * s) + b_x * (s - 1) * (3 * s - 1)
    yp = 3 * y_f * s ** 2 - 3 * y_i * (s - 1) ** 2 + a_y * (3 * s ** 2 - 2 * s) + b_y * (s - 1) * (3 * s - 1)

    v = np.sqrt(xp ** 2 + yp ** 2)
    theta = np.arctan2(yp, xp)

    # Compute the second derivate
    xpp = 6 * x_f * s - 6 * x_i * (s - 1) + a_x * (6 * s - 2) + b_x * (6 * s - 4)
    ypp = 6 * y_f * s - 6 * y_i * (s - 1) + a_y * (6 * s - 2) + b_y * (6 * s - 4)

    # Compute the angular velocity
    w = (ypp * xp - xpp * yp) / (v ** 2)

    out = {}
    out['x'] = x
    out['y'] = y
    out['theta'] = theta
    out['v'] = v * tau
    out['w'] = w * tau
    out['dxdt'] = xp
    out['dydt'] = yp
    out['dxdt2'] = xpp
    out['dydt2'] = ypp

    return out


def plot_wmr(pose, scale=1., ax=None, color=None):
    # Image of triangle in (0,0)
    X = np.array([[0., -1.], [0., 1.], [2., 0.]]) * scale

    x0, y0, theta = pose
    R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    # Matrix of rotation
    X = np.matmul(X, R)

    # Traslate
    Xt = np.vstack((X.transpose(), [1, 1, 1]))
    # 2D Traslation Matrix
    T = np.identity(3)
    T[0:2, 2] = [x0, y0]

    Xtr = np.matmul(T, Xt)
    robot = Xtr[0:-1, :].traspose()

    if ax:
        if not color:
            color = 'black'
        robot_p = plt.Polygon(robot, color=color)
        ax.add_patch(robot_p)
    return robot


def compute_path_from_waypoints(waypoints, k, t=None):
    # Compute Path
    path = {}

    for i in range(len(waypoints) - 1):
        q_i = waypoints[i]
        q_f = waypoints[i + 1]
        path[i] = compute_cubic_trajectory(q_i, q_f, k, t)

    return path


def ref_trajectory_generation(t):
    x_c = -1
    y_c = -1
    R = 10
    w_d = 1 / 15  # desired angular speed

    x = x_c + R * np.sin(2 * w_d * t)
    y = y_c + R * np.sin(w_d * t)

    dotx = 2 * R * w_d * np.cos(2 * w_d * t)
    doty = R * w_d * np.cos(w_d * t)

    return [x, y, dotx, doty]


def plot_path(path):
    plt.plot(path[0]['x'], path[0]['y'], 'r-')
    plt.plot(path[0]['x'][0], path[0]['y'][0], 'g', marker='o', markersize=4)
    plt.plot(path[0]['x'][-1], path[0]['y'][-1], 'g', marker='o', markersize=4)
    plt.grid()
    plt.axis('equal')
    plt.xlabel(r'$x$')
    plt.ylabel(r'$y$')
    plt.show()


if __name__ == '__main__':
    # Initial e Final Pose
    waypoints = np.array([[0., 0., 0.],
                          [3., 3., 0.], [2, 6, 0]
                          ])
    t = np.linspace(0, 30, 1500)

    # d = np.sqrt(2)
    # theta = np.pi + np.pi / 3
    # e_x = d * np.sin(theta)
    # e_y = d * np.cos(theta)
    # x = e_x * np.ones(len(t))
    # y = e_y * np.ones(len(t))
    # Initial and Final Speed
    k = 1
    path = compute_path_from_waypoints(waypoints, k, t)
    # a = np.cos(path[0]['theta']+np.pi) * x - np.sin(path[0]['theta']+np.pi) * y
    # b = np.sin(path[0]['theta']+np.pi) * x + np.cos(path[0]['theta']+np.pi) * y
    # x_error = path[0]['x'] - a
    # y_error = path[0]['y'] - b

    plt.plot(path[0]['x'], path[0]['y'], 'r-')
    plt.plot(path[0]['x'][0], path[0]['y'][0], 'b', marker='o', markersize=4)
    plt.plot(path[0]['x'][-1], path[0]['y'][-1], 'b', marker='o', markersize=4)
    plt.plot(path[1]['x'], path[1]['y'], 'g-')
    plt.plot(path[1]['x'][0], path[1]['y'][0], 'b', marker='o', markersize=4)
    plt.plot(path[1]['x'][-1], path[1]['y'][-1], 'b', marker='o', markersize=4)

    # plt.plot(x_error, y_error, 'g-')

    plt.grid()

    # plt.axis('equal')
    plt.xlabel(r'$x$')
    plt.ylabel(r'$y$')
    plt.show()
    path_dic = ['x', 'y', 'theta', 'v', 'w', 'dxdt', 'dydt','dxdt2','dydt2']
    ans = {'0':3,'1':4}
    tt = [0, 1, 5, 6]
    t1 = np.linspace(0, 30, 1500)
    t2 = np.linspace(30, 60, 1500)

    fig, axs = plt.subplots(2)

    for key,value in zip(ans.keys(),ans.values()):
        key = eval(key)
        axs[key].plot(t1, path[0][path_dic[value]], 'g-')
        axs[key].plot(t1[0], path[0][path_dic[value]][0], 'b', marker='o', markersize=4)
        axs[key].plot(t1[-1], path[0][path_dic[value]][-1], 'b', marker='o', markersize=4)

        axs[key].plot(t2, path[1][path_dic[value]], 'g-')
        axs[key].plot(t2[0], path[1][path_dic[value]][0], 'r', marker='o', markersize=4)
        axs[key].plot(t2[-1], path[1][path_dic[value]][-1], 'y', marker='o', markersize=4)

        axs[key].set(xlabel='t', ylabel=path_dic[value])
        axs[key].grid()
    plt.show()

    fig, axs = plt.subplots(2)
    for i in range(len(waypoints) - 1):
        axs[i].plot(t, path[0][path_dic[i + 5]], 'b--')
        axs[i].plot(t[0], path[0][path_dic[i + 5]][0], 'b', marker='o', markersize=4)
        axs[i].plot(t[-1], path[0][path_dic[i + 5]][-1], 'b', marker='o', markersize=4)
        axs[i].set(xlabel='t', ylabel=path_dic[i + 5])
        axs[i].grid()
    plt.show()

    '''

    plt.figure()
    for i in range(len(waypoints) - 1):
        plt.plot(path[i]['x'], path[i]['y'])
    plt.axis('equal')
    plt.show()

    # Plot the control law
    plt.figure()
    for i in range(len(waypoints) - 1):
        plt.plot(path[i]['v'], path[i]['w'])
    plt.show()
    t = np.linspace(0, 100, 1000)
    (x_d, y_d, dotx_d, doty_d) = ref_trajectory_generation(t)
    # %%
    plt.plot(x_d, y_d)
    plt.title('Desidered Cartesian Trajectory')
    plt.axis('equal')
    plt.grid()
    plt.show()
    '''
