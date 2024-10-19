from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np
import pychrono as chrono

import utils


class SimulationScenarios:
    """
    Simulation scenarios.

    Parameters
    ----------
    vehicle_params : SimpleNamespace
    simulation_params : SimpleNamespace

    """

    def __init__(
        self, vehicle_params: SimpleNamespace, simulation_params: SimpleNamespace
    ) -> None:
        self._vehicle_params = vehicle_params
        self._simulation_params = simulation_params

    @property
    def straight_line_constant_speed(self) -> SimulationScenario:
        """
        Straight line, constant speed driving motors.

        Returns
        -------
        SimulationScenario

        """
        angular_velocity = utils.convert_linear_to_angular_velocity(
            linear_velocity=-1.0, wheel_radius=self._vehicle_params.wheel.radius
        )
        angular_velocity_function = utils.compute_ramp_function(
            transient_time=3.0, y_initial=0.0, y_final=angular_velocity
        )
        return SimulationScenario(
            angular_velocity_left_front=angular_velocity_function,
            angular_velocity_right_front=angular_velocity_function,
            angular_velocity_left_rear=angular_velocity_function,
            angular_velocity_right_rear=angular_velocity_function,
            slope=self._simulation_params.slope,
            friction=self._simulation_params.friction,
        )

    @property
    def steering_motors_no_motion(self) -> SimulationScenario:
        """
        Steering motors activation, no motion.

        Returns
        -------
        SimulationScenario

        """
        transient_time = 3.0
        final_angular_velocity = 0.134
        final_steering_angle = np.pi / 4
        positive_angular_velocity_function = utils.compute_ramp_function(
            transient_time, y_initial=0.0, y_final=final_angular_velocity
        )
        negative_angular_velocity_function = utils.compute_ramp_function(
            transient_time, y_initial=0.0, y_final=-final_angular_velocity
        )
        positive_steering_angle_function = utils.compute_ramp_function(
            transient_time, y_initial=0.0, y_final=final_steering_angle
        )
        negative_steering_angle_function = utils.compute_ramp_function(
            transient_time, y_initial=0.0, y_final=-final_steering_angle
        )
        return SimulationScenario(
            angular_velocity_left_front=negative_angular_velocity_function,
            angular_velocity_right_front=negative_angular_velocity_function,
            angular_velocity_left_rear=positive_angular_velocity_function,
            angular_velocity_right_rear=positive_angular_velocity_function,
            steering_angle_left_front=negative_steering_angle_function,
            steering_angle_right_front=positive_steering_angle_function,
            steering_angle_left_rear=positive_steering_angle_function,
            steering_angle_right_rear=negative_steering_angle_function,
            slope=self._simulation_params.slope,
            friction=self._simulation_params.friction,
            total_time=transient_time,
        )


@dataclass
class SimulationScenario:
    """
    Simulation scenario.

    Notes
    -----
    angle : deg
    angular_velocity : rad/s
    gravity : m/s²

    """

    angular_velocity_left_front: chrono.ChFunction
    angular_velocity_right_front: chrono.ChFunction
    angular_velocity_left_rear: chrono.ChFunction
    angular_velocity_right_rear: chrono.ChFunction
    steering_angle_left_front: chrono.ChFunction = chrono.ChFunction_Const(0.0)
    steering_angle_right_front: chrono.ChFunction = chrono.ChFunction_Const(0.0)
    steering_angle_left_rear: chrono.ChFunction = chrono.ChFunction_Const(0.0)
    steering_angle_right_rear: chrono.ChFunction = chrono.ChFunction_Const(0.0)
    are_towing_wheel: bool = False
    slope: float = 0.0
    friction: float = 0.7
    damping: float = 10.0
    total_time: float = 10.0
    time_step: float = 5.0e-3
    gravity_vector: chrono.ChVectorD = field(init=False)
    gravity: float = 9.81

    def __post_init__(self) -> None:
        self.gravity_vector = self._compute_gravity_vector()

    def _compute_gravity_vector(self) -> chrono.ChVectorD:
        """
        Compute gravity vector (constant-slope ramp).

        Returns
        -------
        chrono.ChVectorD

        """
        slope = np.deg2rad(self.slope)
        x = self.gravity * np.sin(slope)
        z = self.gravity * np.cos(slope)
        return chrono.ChVectorD(-x, 0.0, -z)
