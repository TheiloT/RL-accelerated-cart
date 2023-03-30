import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(predicted_x=None, gt_x=None, T=1, crop_y=False):
    """
    Plots the ground truth trajectory vs the predicted trajectory on a (t, x) plot.
        :param predicted_x: Regularly spaced samples of the predicted trajectory. If None, this trajectory is not plotted.
        :type predicted_x: np.ndarray[float], optional
        :param gt_x: Regularly spaced samples of the ground truth trajectory. If None, this trajectory is not plotted.
        :type gt_x: np.ndarray[float], optional
        :param float|int T: Time horizon.
        :param bool crop_y: If True, crop the y axis to zoom on [-1, 0].
    """
    plt.figure()
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
    plt.grid()
    plt.show()


def plot_control(predicted_u=None, gt_u=None,  T=1):
    """
    Plots the ground truth control vs the predicted control on a (t, u) plot.
        :param predicted_u: Regularly spaced samples of the predicted control. If None, this control is not plotted.
        :type predicted_u: np.ndarray[float], optional
        :param gt_u: Regularly spaced samples of the ground truth control. If None, this control is not plotted.
        :type gt_u: np.ndarray[float], optional
        :param float|int T: Time horizon.
    """
    plt.figure()
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
    plt.grid()
    plt.show()
