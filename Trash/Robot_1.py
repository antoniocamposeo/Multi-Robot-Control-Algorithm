import numpy as np
import matplotlib.pyplot as plt
import cubicpath
from scipy.integrate import odeint


class Robot_1:
    def __init__(self, name, r_to_f, start_position, b, k_1, k_2, k_1f, k_2f):
        self.name = name
        self.r_to_f = r_to_f
        self.start_position = start_position
        self.robot_states = []
        self.k_1 = k_1
        self.k_2 = k_2
        self.k_1f = k_1f
        self.k_2f = k_2f
        self.b = b
        self.u1 = []
        self.u2 = []
        self.theta_t = []
        self.sim_t = []
        self.doty_d = None
        self.dotx_d = None
        self.x_d = None
        self.y_d = None
        self.zeta = 1
        self.a = 5
        self.path = []

    def k3(self, v_d, w_d):
        return 2 * self.zeta * np.sqrt(v_d ** 2 + self.a * w_d ** 2)

    def k1(self, v_d, w_d):
        return 2 * self.zeta * np.sqrt(v_d ** 2 + self.a * w_d ** 2)

    def model(self, y, t, y1d, y2d, doty1d, doty2d, time):
        T = np.searchsorted(time, t)
        if T >= len(time):
            T = len(time) - 1
        y1 = y[0]
        y2 = y[1]
        theta = y[2]
        (u_1, u_2) = self.control(y1, y2, y1d[T], y2d[T], doty1d[T], doty2d[T])
        doty_1 = u_1
        doty_2 = u_2
        dottheta = u_2 / self.b * np.cos(theta) - u_1 / self.b * np.sin(theta)
        self.theta_t.append(theta)
        self.u1.append(u_1)
        self.u2.append(u_2)
        self.sim_t.append(t)
        return [doty_1, doty_2, dottheta]

    def model_1(self, y, t, path, time):
        # T is the index in the 'time' vector that is
        # closest to the current integration time 't'
        T = np.searchsorted(time, t)
        if (T >= len(time)):
            T = len(time) - 1
        y1 = y[0]
        y2 = y[1]
        theta = y[2]
        e = [y1 - path[0]['x'][T], y2 - path[0]['y'][T], theta - path[0]['theta'][T]]
        v_d = path[0]['v']
        w_d = path[0]['w']
        # e[1] is e_2
        u_1, u_2 = self.control_1(e, path[0]['v'][T], path[0]['w'][T])
        self.u1.append(u_1)
        self.u2.append(u_2)
        edot_1 = u_1 + e[1] * (w_d[T] - u_2)
        edot_2 = v_d[T] * np.sin(e[2]) - e[0] * (w_d[T] - u_2)
        edot_3 = u_2
        return [edot_1, edot_2, edot_3]

    def control_1(self, e, v_d, w_d):
        #k2 = 10
        k2 = self.a
        u_1 = - self.k1(v_d, w_d) * e[0]

        if e[2] == 0:
            u_2 = -k2 * v_d * e[1] - self.k3(v_d, w_d) * e[2]
        else:
            u_2 = -k2 * v_d * np.sin(e[2]) / e[2] * e[1] - self.k3(v_d, w_d) * e[2]

        return np.array([u_1, u_2])

    def control(self, y1, y2, y1d, y2d, doty1d, doty2d):
        u_1 = self.k_1f * doty1d + self.k_1 * (y1d - y1)
        u_2 = self.k_2f * doty2d + self.k_2 * (y2d - y2)
        return [u_1, u_2]

    def compute_distance(self):
        pass

    def ref_trajectory_generation(self, t):
        # linear trajectory
        if self.name == 'Leader':
            self.x_d = t
            self.y_d = 0 * np.eye(1, len(t))[0]
            self.dotx_d = 0 * np.eye(1, len(t))[0]
            self.doty_d = 0 * np.eye(1, len(t))[0]
        elif self.name == 'Follower':
            self.x_d = self.r_to_f.robot_states[:, 0] - 0.3
            self.y_d = 0 * np.eye(1, len(t))[0]
            self.dotx_d = 0 * np.eye(1, len(t))[0]
            self.doty_d = 0 * np.eye(1, len(t))[0]

    def compute_control_odeint(self, t):
        self.ref_trajectory_generation(t)
        self.robot_states = odeint(self.model, np.array(self.start_position), t,
                                   args=(self.x_d, self.y_d, self.dotx_d, self.doty_d, t))
        return self.robot_states

    def compute_control_1_odeint(self, t, k):
        waypoints = np.array([[0., 0., 0.], [10, 10, 0]])
        self.path = cubicpath.compute_path_from_waypoints(waypoints, k, t)
        self.robot_states = odeint(self.model_1, np.array(self.start_position), t,
                                   args=(self.path, t))
        return self.robot_states
