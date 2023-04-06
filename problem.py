import numpy as np
from params import *


# Cost

def cost(x, x_dot, u):
    """Approximation of the cost function evaluated in a given trajectory-control pair (x, u), using trapezoidal rule.
    
    :param np.ndarray[float] x: The trajectory.
    :param np.ndarray[float] x_dot: The temporal derivative of the trajectory.
    :param np.ndarray[float] u: The control.
    :return float: Approximation of the cost.
    """
    return LAMBDA_P*x[-1]**2 + LAMBDA_V*x_dot[-1]**2 + np.trapz(u**2, dx=T/u.shape[0])


# State dynamics of the system

def  dynamics(x, v, u, dt=DT):
    """Takes position and action ((x, v), a) and returns a new state x_new.
    
    :param float x: Initial position.
    :param float v: Initial velocity.
    :param float u: Action, considered cosntant on the integration time.
    :param float dt: Integration time.
    :return (float, float): The new state x_new.
    """
    return x + dt*v + dt**2/(2*M_REAL)*u, v + dt/M_REAL*u

def simulator(x_0, policy, dt=DT):
    """Run a simulation of the system between times 0 and T.
    
    :param x_0: Initial positioni in [0, 1].
    :policy function: Function that given a position and time returns an action.
    :dt float: The integration time between two samples. 
    :return: The triplet (x,  r) where x is the resulting trajectory, u the control and r the reward history, which are 3 np.arrays of size (N+1).
    """
    n_samples = int(T/dt)
    x = np.zeros(n_samples+1)
    x[0] = x_0
    v = 0
    for n in range(n_samples):
        u = policy(n*dt, x[n])
        x[n+1], v = dynamics(x[n], v, u, dt=dt)
    return x
