import numpy as np
from params import *


def ground_truth(x_0, t):
    """Analytical expressions of the trajectory, its derivative and the control for the continuous control problem.
    
    :param float x_0: Initial condition in [-1, 0].
    :return: The ground truth solution, its derivative and the ground truth control.
    :rtype: 3-tuple of float.
    """
    C1 = 1 + (LAMBDA_P*T**3)/(3*M_REAL**2)
    C2 = (LAMBDA_P*T**2)/(2*M_REAL**2)
    C3 = (LAMBDA_V*T**2)/(2*M_REAL**2)
    C4 = 1 + (LAMBDA_V*T)/(M_REAL**2)
    p1 = (2*C4)/(C1*C4-C2*C3)*LAMBDA_P*x_0
    p2 = -(2*C3)/(C1*C4-C2*C3)*LAMBDA_P*x_0
    x = p1/(12*M_REAL**2)*(T**3 - (T-t)**3) - p2/(4*M_REAL**2)*t**2 - (T**2*p1)/(4*M_REAL**2)*t + x_0
    v = p1/(4*M_REAL**2)*(T-t)**2 - p2/(2*M_REAL**2)*t - (T**2*p1)/(4*M_REAL**2)
    u = -1/(2*M_REAL)*(p1*(T-t) + p2)
    return x, v, u


def ground_truth_sample(x_0, res=10000):
    """Samples the analytical trajectory, its derivative and the analytical control for the continuous control problem.
    
    :param float x_0: Initial condition in [-1, 0].
    :param int res: Number of points to sample.
    :return: The sampled solution and control.
    :rtype: 3-tuple of np.ndarray[float] all of shape (res).
    """
    t = np.linspace(0, T, res)
    return ground_truth(x_0, t)
