import numpy as np
import matplotlib.pyplot as plt
from params import *


# Plotting

def plot_trajectory(predicted_x=None, gt_x=None, crop_y=False):
    """Plots the ground truth trajectory vs the predicted trajectory on a (t, x) plot.
    
    :param predicted_x: Regularly spaced samples of the predicted trajectory. If None, this trajectory is not plotted.
    :type predicted_x: np.ndarray[float], optional
    :param gt_x: Regularly spaced samples of the ground truth trajectory. If None, this trajectory is not plotted.
    :type gt_x: np.ndarray[float], optional
    :param bool crop_y: If True, crop the y axis to zoom on [-1, 0].
    """
    plt.figure()
    plt.grid()
    if predicted_x is not None:
        t_prediction = np.linspace(0, T, predicted_x.shape[0])
        plt.plot(t_prediction, predicted_x, label="Prediction", color="tab:blue")
    if gt_x is not None:
        t_gt = np.linspace(0, T, gt_x.shape[0])
        plt.plot(t_gt, gt_x, label="Ground truth", color="tab:orange")
    if crop_y:
        plt.ylim([-1, 0])
    plt.xlabel("t")
    plt.ylabel("x")
    plt.title("Trajectories")
    plt.legend()
    plt.show()


def plot_control(predicted_u=None, gt_u=None,  T=1):
    """Plots the ground truth control vs the predicted control on a (t, u) plot.
    
    :param predicted_u: Regularly spaced samples of the predicted control. If None, this control is not plotted.
    :type predicted_u: np.ndarray[float], optional
    :param gt_u: Regularly spaced samples of the ground truth control. If None, this control is not plotted.
    :type gt_u: np.ndarray[float], optional
    """
    plt.figure()
    plt.grid()
    if predicted_u is not None:
        t_prediction = np.linspace(0, T, predicted_u.shape[0])
        plt.scatter(t_prediction, predicted_u, label="Prediction", color="tab:blue", marker="x")
    if gt_u is not None:
        t_gt = np.linspace(0, T, gt_u.shape[0])
        plt.plot(t_gt, gt_u, label="Ground truth", color="tab:orange")
    plt.xlabel("t")
    plt.ylabel("u")
    plt.title("Controls")
    plt.legend()
    plt.show()


def plot_V_x(V, v=0, state_zoom=None, colorbar_limits=None, zoom_time=0):
    """Plots the value function on an (x, n) axis (where n is the time step) for a fixed velocity v.
    
    :param V: The value function to plot.
    :type V: np.ndarray[float] of dimension N x ((X_R-X_L)*N_X+1) x ((V_R-V_L)*N_V+1).
    :param float v: The fixed velocity at which to plot V.
    :param np.ndarray[float] state_zoom: Values of V for states smaller or greater than the boundaries of this state window are not plotted. If None, takes the whole state space.
    :param np.ndarray[float] colorbar_limits: Sets the min and max boundaries for the colorbar; values of V above these boundaries are capped to the nearest boundary.
    :param int zoom_time: The distance between 2 ticks on the n axis is multiplied by 2^(zoom_time), to make the plot more readable. Otherwise, the plot might be too streched in x.
    """
    state_zoom = state_zoom if state_zoom else (0, V.shape[1]-1)
    colorbar_limits = colorbar_limits if colorbar_limits else (None, None)
    truncated_V = V[:,state_zoom[0]:state_zoom[1]+1, to_arr_v(v)]
    truncated_V = np.expand_dims(truncated_V, axis=1)
    for _ in range(zoom_time):
        truncated_V = np.concatenate([truncated_V, truncated_V] , axis=1)
    truncated_V = np.reshape(truncated_V, [truncated_V.shape[0]*truncated_V.shape[1], truncated_V.shape[2]])
    plt.figure()
    plt.imshow(truncated_V, vmin=colorbar_limits[0], vmax=colorbar_limits[1])
    xticks = np.append(np.arange(0, truncated_V.shape[1], truncated_V.shape[1]//5), (to_arr_x(0)-state_zoom[0]))
    plt.xticks(ticks=xticks, labels=[f"{x:.2f}" for x in from_arr_x(np.arange(state_zoom[0], state_zoom[1]+1, truncated_V.shape[1]//5))]+[0])
    yticks = np.arange(0, truncated_V.shape[0]+1, truncated_V.shape[0]//2)
    plt.yticks(ticks=yticks, labels=[n/(2**zoom_time) for n in yticks])
    plt.colorbar(location="bottom")
    plt.title("Value function")
    plt.xlabel("x")
    plt.ylabel("n")
    plt.show()



# Conversion from float to array indices

def transform_interval(a, b, c, d, x):
    """Affine transformation that maps an interval [a, b] to an interval [c, d].
    
    :param int a: Lower bound of the departure interval.
    :param int b: Upper bound of the departure interval.
    :param int c: Lower bound of the departure interval.
    :param int d: Upper bound of the departure interval.
    :param float x: The point to map from [a, b] to [c, d].
    :return float: The point of [c, d] that is the image of x by the affine transformation.
    """
    return (d-c)/(b-a)*x + (c*b-d*a)/(b-a)

to_arr_x = lambda x: round(transform_interval(X_L, X_R, 0, (X_R-X_L)*N_X, x))  # This is to convert a position to an index for value functions and policies
from_arr_x = lambda x: transform_interval(0, (X_R-X_L)*N_X, X_L, X_R, x)  # Inverse of the previous
to_arr_v = lambda v: round(transform_interval(V_L, V_R, 0, (V_R-V_L)*N_V, v))  # This is to convert a velocity to an index for value functions and policies
from_arr_v = lambda v: transform_interval(0, (V_R-V_L)*N_V, V_L, V_R, v)  # Inverse of the previous
to_arr_a = lambda a: round(transform_interval(U_L, U_R, 0, (U_R-U_L)*N_U, a))  # This is to convert an action to an index for value functions and policies
from_arr_a = lambda a: transform_interval(0, (U_R-U_L)*N_U, U_L, U_R, a)  # Inverse of the previous
