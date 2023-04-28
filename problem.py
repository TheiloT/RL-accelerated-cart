import numpy as np
import params as p


# Cost

def cost(x, x_dot, u):
    """Approximation of the cost function evaluated in a given trajectory-control pair (x, u), using trapezoidal rule.
    
    :param np.ndarray[float] x: The trajectory.
    :param np.ndarray[float] x_dot: The temporal derivative of the trajectory.
    :param np.ndarray[float] u: The control.
    :return float: Approximation of the cost.
    """
    return p.LAMBDA_P*x[-1]**2 + p.LAMBDA_V*x_dot[-1]**2 + np.trapz(u**2, dx=p.T/u.shape[0])



# State dynamics of the system

def dynamics(x, v, u):
    """Takes position and action ((x, v), u) and returns a new state x_new.
    
    :param float x: Current position.
    :param float v: Current velocity.
    :param float u: Action, considered cosntant on the integration time.
    :return (float, float): New state and velocity.
    """
    return x + p.DT*v + p.DT**2/(2*p.M_REAL)*u, v + p.DT/p.M_REAL*u


def simulator(x_0, policy):
    """Run a simulation of the system between times 0 and T.
    
    :param float x_0: Initial positioni in [0, 1].
    :param function policy: Function that given a position, velocity and time returns an action.
    :param float dt: The integration time between two samples. 
    :return: The couple (x, u) where x is the resulting trajectory and u the control which are 2 np.arrays of size (N+1) and (N) respectively, where N=T/dt.
    """
    x = np.zeros(p.N+1)
    u = np.zeros(p.N)
    x[0] = x_0
    v = 0
    for n in range(p.N):
        u[n] = policy(n*p.DT, x[n], v)
        x[n+1], v = dynamics(x[n], v, u[n])
    return x, u
