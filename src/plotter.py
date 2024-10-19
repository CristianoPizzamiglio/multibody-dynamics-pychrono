from __future__ import annotations

from matplotlib import pyplot as plt

import utils
from simulation import Results


class Plotter:
    """
    Plotter.

    Parameters
    ----------
    results : Results

    """

    _sampling = 10

    def __init__(self, results: Results) -> None:
        self._results = results

    def plot_driving_motor_torques(self) -> None:
        """Plot driving motor torques."""

        fig, ax = plt.subplots(2, 2)
        fig.suptitle("Driving Motor Torques (Nm)")

        ax[0, 0].plot(self._results.driving_motor_left_front_torque[:: self._sampling])
        ax[0, 0].set_title("Left-Front")

        ax[0, 1].plot(self._results.driving_motor_right_front_torque[:: self._sampling])
        ax[0, 1].set_title("Right-Front")

        ax[1, 0].plot(self._results.driving_motor_left_rear_torque[:: self._sampling])
        ax[1, 0].set_title("Left-Rear")

        ax[1, 1].plot(self._results.driving_motor_right_rear_torque[:: self._sampling])
        ax[1, 1].set_title("Right-Rear")

        plt.subplots_adjust(wspace=0.3, hspace=0.5)
        plt.show()

    def plot_driving_motor_torques_smoothed(self) -> None:
        """Plot smoothed driving motor torques."""

        fig, ax = plt.subplots(2, 2)
        fig.suptitle("Driving Motor Torques Smoothed (Nm)")

        ax[0, 0].plot(
            utils.smooth_data(
                self._results.driving_motor_left_front_torque[:: self._sampling]
            )
        )
        ax[0, 0].set_title("Left-Front")

        ax[0, 1].plot(
            utils.smooth_data(
                self._results.driving_motor_right_front_torque[:: self._sampling]
            )
        )
        ax[0, 1].set_title("Right-Front")

        ax[1, 0].plot(
            utils.smooth_data(
                self._results.driving_motor_left_rear_torque[:: self._sampling]
            )
        )
        ax[1, 0].set_title("Left-Rear")

        ax[1, 1].plot(
            utils.smooth_data(
                self._results.driving_motor_right_rear_torque[:: self._sampling]
            )
        )
        ax[1, 1].set_title("Right-Rear")

        plt.subplots_adjust(wspace=0.3, hspace=0.5)
        plt.show()

    def plot_steering_motor_torques(self) -> None:
        """Plot steering motor torques."""

        fig, ax = plt.subplots(2, 2)
        fig.suptitle("Steering Motor Torques (Nm)")

        ax[0, 0].plot(self._results.steering_motor_left_front_torque[:: self._sampling])
        ax[0, 0].set_title("Left-Front")

        ax[0, 1].plot(
            self._results.steering_motor_right_front_torque[:: self._sampling]
        )
        ax[0, 1].set_title("Right-Front")

        ax[1, 0].plot(self._results.steering_motor_left_rear_torque[:: self._sampling])
        ax[1, 0].set_title("Left-Rear")

        ax[1, 1].plot(self._results.steering_motor_right_rear_torque[:: self._sampling])
        ax[1, 1].set_title("Right-Rear")

        plt.subplots_adjust(wspace=0.3, hspace=0.5)
        plt.show()

    def plot_steering_motor_torques_smoothed(self) -> None:
        """Plot smoothed steering motor torques."""

        fig, ax = plt.subplots(2, 2)
        fig.suptitle("Steering Motor Torques Smoothed (Nm)")

        ax[0, 0].plot(
            utils.smooth_data(
                self._results.steering_motor_left_front_torque[:: self._sampling]
            )
        )
        ax[0, 0].set_title("Left-Front")

        ax[0, 1].plot(
            utils.smooth_data(
                self._results.steering_motor_right_front_torque[:: self._sampling]
            )
        )
        ax[0, 1].set_title("Right-Front")

        ax[1, 0].plot(
            utils.smooth_data(
                self._results.steering_motor_left_rear_torque[:: self._sampling]
            )
        )
        ax[1, 0].set_title("Left-Rear")

        ax[1, 1].plot(
            utils.smooth_data(
                self._results.steering_motor_right_rear_torque[:: self._sampling]
            )
        )
        ax[1, 1].set_title("Right-Rear")

        plt.subplots_adjust(wspace=0.3, hspace=0.5)
        plt.show()
