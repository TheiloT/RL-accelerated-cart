import numpy as np
from params import *


def ground_truth(x_0, t):
    """Analytical expressions of the trajectory, its derivative and the control for the continuous control problem.
    
    :param float x_0: Initial condition in [-1, 0].
    :return: The ground truth solution, its derivative and the ground truth control.
    :rtype: 3-tuple of float.
    """
    D = LAMBDA_P*T**3 + 12*M_REAL**2
    return (
        (2*LAMBDA_P*x_0/D)*t**3 - (3*LAMBDA_P*T*x_0/D)*t**2 + x_0,
        (6*LAMBDA_P*x_0/D)*t**2 - (6*LAMBDA_P*T*x_0/D)*t,
        (12*M_REAL*LAMBDA_P*x_0/D)*t - (6*M_REAL*LAMBDA_P*T*x_0/D)
    )

def ground_truth_sample(x_0, res=10000):
    """Samples the analytical trajectory, its derivative and the analytical control for the continuous control problem.
    
    :param float x_0: Initial condition in [-1, 0].
    :param int res: Number of points to sample.
    :return: The sampled solution and control.
    :rtype: 3-tuple of np.ndarray[float] all of shape (res).
    """
    t = np.linspace(0, T, res)
    return ground_truth(x_0, t)
