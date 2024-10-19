from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import pychrono as chrono
import pychrono.irrlicht as irr

from simulation_scenarios import SimulationScenario
from vehicle import Vehicle


class Simulation:
    """
    Simulation.

    Parameters
    ----------
    system : chrono.ChSystem
    visual_system : irr.ChVisualSystemIrrlicht
    vehicle : Vehicle
    simulation_scenario : SimulationScenario

    """

    def __init__(
        self,
        system: chrono.ChSystem,
        visual_system: irr.ChVisualSystemIrrlicht,
        vehicle: Vehicle,
        simulation_scenario: SimulationScenario,
    ) -> None:
        self._system = system
        self._visual_system = visual_system
        self._vehicle = vehicle
        self._simulation_scenario = simulation_scenario
        self.results = Results()

    def run(self) -> None:
        """Run simulation."""

        self._system.SetMinBounceSpeed(0.1000)
        self._system.SetMaxiter(200)
        while self._visual_system.Run():
            self._visual_system.BeginScene()
            self._visual_system.Render()
            self._visual_system.EndScene()
            self._system.DoStepDynamics(self._simulation_scenario.time_step)
            self._append_results(self._vehicle, self.results)
            if self._system.GetChTime() > self._simulation_scenario.total_time:
                self._visual_system.GetDevice().closeDevice()

    @staticmethod
    def _append_results(vehicle: Vehicle, results: Results) -> None:
        """
        Append results.

        Parameters
        ----------
        vehicle : Vehicle
        results : Results

        """
        results.driving_motor_left_front_torque.append(
            vehicle.mobility_subsystem_left_front.driving_motor.GetMotorTorque()
        )
        results.driving_motor_right_front_torque.append(
            vehicle.mobility_subsystem_right_front.driving_motor.GetMotorTorque()
        )
        results.driving_motor_left_rear_torque.append(
            vehicle.mobility_subsystem_left_rear.driving_motor.GetMotorTorque()
        )
        results.driving_motor_right_rear_torque.append(
            vehicle.mobility_subsystem_right_rear.driving_motor.GetMotorTorque()
        )
        results.steering_motor_left_front_torque.append(
            vehicle.mobility_subsystem_left_front.steering_motor.GetMotorTorque()
        )
        results.steering_motor_right_front_torque.append(
            vehicle.mobility_subsystem_right_front.steering_motor.GetMotorTorque()
        )
        results.steering_motor_left_rear_torque.append(
            vehicle.mobility_subsystem_left_rear.steering_motor.GetMotorTorque()
        )
        results.steering_motor_right_rear_torque.append(
            vehicle.mobility_subsystem_right_rear.steering_motor.GetMotorTorque()
        )


@dataclass
class Results:
    """Results."""

    driving_motor_left_front_torque: List[float] = field(default_factory=list)
    driving_motor_right_front_torque: List[float] = field(default_factory=list)
    driving_motor_left_rear_torque: List[float] = field(default_factory=list)
    driving_motor_right_rear_torque: List[float] = field(default_factory=list)
    steering_motor_left_front_torque: List[float] = field(default_factory=list)
    steering_motor_right_front_torque: List[float] = field(default_factory=list)
    steering_motor_left_rear_torque: List[float] = field(default_factory=list)
    steering_motor_right_rear_torque: List[float] = field(default_factory=list)
