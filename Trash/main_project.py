import numpy as np

from Project.robot import Robot
from Project.control import *
from Project.cubicpath import *
from Project.utils import *

R1 = Robot('Leader')
t = np.linspace(0, 20, 1000)
waypoints = np.array([[1,1, np.pi], [20, 20., np.pi]])
# Initial and Final Speed
k = 2
path = compute_path_from_waypoints(waypoints, k,t)

###############################################
''' Set value of gains C1 - a , zeta , k_2 '''
C1 = TrajectoryTracking_Control(4, 1, 4)

C2 = I_O_Linearization_Control(1,1,0.1,0.1)
C2.set_b(0.2)

C3 = CartesianRegulator_Control(0.5, 0.5)

C4 = PolarRegulator_Control(1,0.3,0.3)

######################################################
R1.set_control_obj(C2)
R1.set_initial_state([1,1,0])
R1.set_time(t)
R1.set_path(path)

######################################################
R1.simulation()
plot_xy(R1.get_x(), R1.get_y(), path, None, None)
