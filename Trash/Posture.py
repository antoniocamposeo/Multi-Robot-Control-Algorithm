import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib

# Magic to enable matplotlib widgets
plt.rcParams['figure.figsize'] = [10 * 2 / 2.54, 8 * 2 / 2.54]
matplotlib.rcParams['figure.dpi'] = 160


# %%

# Posture Regulation
class UnicycleControl:
    def __init__(self):
        self.t = None
        self.q0 = None
        self.v = None
        self.w = None
        self.sim_args = None
        self.x_d = None
        self.y_d = None
        self.theta_d = None
        self.w_d = None
        self.v_d = None

    def control(self, *args):
        v = self.control_v(args)
        w = self.control_w(args)
        return v, w

    def set_simulation_args(self, args):
        self.sim_args = args

    def get_simulation_args(self):
        return self.sim_args

    def control_v(self, *args):
        pass

    def control_w(self, *args):
        pass

    def animation(self):
        pass

    def postprocess(self, *args):
        pass

    def get_time(self):
        return self.t

    def set_time(self, t):
        self.t = t

    def set_initial_conditions(self, q0):
        self.q0 = q0

    def get_initial_conditions(self):
        return self.q0

    def dynamics(self, q, t, args):
        pass

    def run_simulation(self):
        q0 = self.get_initial_conditions()
        t = self.get_time()
        args = self.get_simulation_args()
        if q0 is None or t is None:
            print('Error: the intial conditions are not set.')
            raise ValueError

        if args is None:
            self.q = odeint(self.dynamics, q0, t)
        else:
            self.q = odeint(self.dynamics, q0, t, args)

    def plot_xy(self):
        plt.figure()
        plt.plot(self.q[:, 0], self.q[:, 1], 'b')
        plt.grid()
        plt.xlabel(r'$x$')
        plt.ylabel(r'$y$')
        plt.show()


# %%
class CartesianRegulator(UnicycleControl):
    def __init__(self, k_1, k_2):
        super(CartesianRegulator, self).__init__()
        self.k_1 = k_1
        self.k_2 = k_2
        self.v_arr = []
        self.w_arr = []

    def control_v(self, *args):
        (x, y, theta) = args[0]
        k_1 = self.k_1
        v = - k_1 * (x * np.cos(theta) + y * np.sin(theta))
        return v

    def control_w(self, *args):
        (x, y, theta) = args[0]
        k_2 = self.k_2
        w = k_2 * (np.arctan2(y, x) + np.pi - theta)
        return w

    def dynamics(self, q, t):
        x = q[0]
        y = q[1]
        theta = q[2]
        # v = self.control_v(q)
        # w = self.control_w(q)
        v, w = self.control(x, y, theta)
        dotx = v * np.cos(theta)
        doty = v * np.sin(theta)
        dottheta = w
        self.v_arr.append(v)
        self.w_arr.append(w)

        return np.array([dotx, doty, dottheta])

    def get_v(self):
        return self.v_arr

    def get_w(self):
        return self.w_arr


# %%
# Simulation

if __name__ == '__main__':
    q0 = np.array([1, 1, np.pi / 2])
    t = np.linspace(0, 30, 1000)
    k_1 = 0.5
    k_2 = 1
    c = CartesianRegulator(k_1, k_2)
    c.set_time(t)  # sets the simulation time
    c.set_initial_conditions(q0)  # sets the initial conditions
    c.run_simulation()
    # c.plot_xy()
    print(len(c.get_w()))
