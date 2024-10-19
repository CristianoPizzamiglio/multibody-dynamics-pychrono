from __future__ import annotations

from functools import cached_property
from types import SimpleNamespace

import pychrono as chrono

import utils
from simulation_scenarios import SimulationScenario


class Vehicle:
    """
    Vehicle.

    Parameters
    ----------
    params : SimpleNamespace,
    material : chrono.ChMaterialSurface
    simulation_scenario : SimulationScenario

    """

    def __init__(
        self,
        params: SimpleNamespace,
        material: chrono.ChMaterialSurface,
        simulation_scenario: SimulationScenario,
    ) -> None:
        self._params = params
        self._material = material
        self._simulation_scenario = simulation_scenario

    @cached_property
    def chassis(self) -> chrono.ChBodyEasyBox:
        density = utils.compute_box_density(
            self._params.chassis.length,
            self._params.chassis.width,
            self._params.chassis.height,
            self._params.chassis.mass,
        )
        chassis = chrono.ChBodyEasyBox(
            self._params.chassis.length,
            self._params.chassis.width,
            self._params.chassis.height,
            density,
            True,
        )
        chassis.SetPos(
            chrono.ChVectorD(
                self._params.chassis.x_cg,
                self._params.chassis.y_cg,
                self._params.chassis.z_cg,
            )
        )
        chassis.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
        chassis.SetName("Chassis")
        return chassis

    @cached_property
    def mobility_subsystem_left_front(self) -> MobilitySubsystem:
        """
        Mobility subsystem left-front.

        Returns
        -------
        MobilitySubsystem

        """
        return MobilitySubsystem(
            self._params,
            self._material,
            self._simulation_scenario,
            self.chassis,
            is_left=True,
            is_front=True,
            angular_velocity_function=(
                self._simulation_scenario.angular_velocity_left_front
            ),
            steering_angle_function=self._simulation_scenario.steering_angle_left_front,
        )

    @cached_property
    def mobility_subsystem_right_front(self) -> MobilitySubsystem:
        """
        Mobility subsystem right-front.

        Returns
        -------
        MobilitySubsystem

        """
        return MobilitySubsystem(
            self._params,
            self._material,
            self._simulation_scenario,
            self.chassis,
            is_left=False,
            is_front=True,
            angular_velocity_function=(
                self._simulation_scenario.angular_velocity_right_front
            ),
            steering_angle_function=(
                self._simulation_scenario.steering_angle_right_front
            ),
        )

    @cached_property
    def mobility_subsystem_left_rear(self) -> MobilitySubsystem:
        """
        Mobility subsystem left-rear.

        Returns
        -------
        MobilitySubsystem

        """
        return MobilitySubsystem(
            self._params,
            self._material,
            self._simulation_scenario,
            self.chassis,
            is_left=True,
            is_front=False,
            angular_velocity_function=(
                self._simulation_scenario.angular_velocity_left_rear
            ),
            steering_angle_function=self._simulation_scenario.steering_angle_left_rear,
        )

    @cached_property
    def mobility_subsystem_right_rear(self) -> MobilitySubsystem:
        """
        Mobility subsystem right-rear.

        Returns
        -------
        MobilitySubsystem

        """
        return MobilitySubsystem(
            self._params,
            self._material,
            self._simulation_scenario,
            self.chassis,
            is_left=False,
            is_front=False,
            angular_velocity_function=(
                self._simulation_scenario.angular_velocity_right_rear
            ),
            steering_angle_function=self._simulation_scenario.steering_angle_right_rear,
        )

    def add_to_system(self, system: chrono.ChSystem) -> None:
        """
        Add components to system.

        Parameters
        ----------
        system : chrono.ChSystem

        """

        system.Add(self.chassis)
        self.mobility_subsystem_left_front.add_to_system(system)
        self.mobility_subsystem_right_front.add_to_system(system)
        self.mobility_subsystem_left_rear.add_to_system(system)
        self.mobility_subsystem_right_rear.add_to_system(system)


class MobilitySubsystem:
    """
    Mobility subsystem.

    Parameters
    ----------
    params : SimpleNamespace,
    material : chrono.ChMaterialSurface
    simulation_scenario : SimulationScenario
    chassis : chrono.ChBodyEasyBox
    is_left : bool
    is_front : bool
    angular_velocity_function : chrono.ChFunction
    steering_angle_function : chrono.ChFunction

    """

    def __init__(
        self,
        params: SimpleNamespace,
        material: chrono.ChMaterialSurface,
        simulation_scenario: SimulationScenario,
        chassis: chrono.ChBodyEasyBox,
        is_left: bool,
        is_front: bool,
        angular_velocity_function: chrono.ChFunction,
        steering_angle_function: chrono.ChFunction,
    ) -> None:
        self._params = params
        self._material = material
        self._simulation_scenario = simulation_scenario
        self._chassis = chassis
        self._is_left = is_left
        self._is_front = is_front
        self._angular_velocity_function = angular_velocity_function
        self._steering_angle_function = steering_angle_function

    @property
    def _wheel_x_cg(self) -> float:
        """
        Wheel center x-coordinate.

        Returns
        -------
        float

        """
        return self._params.wheel.x_cg if self._is_front else -self._params.wheel.x_cg

    @property
    def _wheel_y_cg(self) -> float:
        """
        Wheel center y-coordinate.

        Returns
        -------
        float

        """
        return self._params.wheel.y_cg if self._is_left else -self._params.wheel.y_cg

    @cached_property
    def wheel(self) -> chrono.ChBodyEasyCylinder:
        """
        Wheel.

        Returns
        -------
        chrono.ChBodyEasyCylinder

        """
        density = utils.compute_cylinder_density(
            self._params.wheel.radius,
            self._params.wheel.width,
            self._params.wheel.mass,
        )
        wheel = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y,
            self._params.wheel.radius,
            self._params.wheel.width,
            density,
            True,
            True,
            self._material,
        )
        wheel.GetVisualShape(0).SetColor(chrono.ChColor(0.0, 0.0, 0.0))
        wheel.GetVisualShape(0).SetOpacity(0.8)
        wheel.SetPos(
            chrono.ChVectorD(
                self._wheel_x_cg, self._wheel_y_cg, self._params.wheel.z_cg
            )
        )
        wheel.SetName("Wheel")
        return wheel

    @cached_property
    def hub(self) -> chrono.ChBodyEasyCylinder:
        """
        Hub.

        Returns
        -------
        chrono.ChBodyEasyCylinder

        """
        density = utils.compute_cylinder_density(
            self._params.hub.radius,
            self._params.hub.length,
            self._params.hub.mass,
        )
        hub = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y,
            self._params.hub.radius,
            self._params.hub.length,
            density,
            True,
            True,
            self._material,
        )
        hub.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
        offset = 0.174
        y_cg = (
            self._params.wheel.y_cg - offset
            if self._is_left
            else offset - self._params.wheel.y_cg
        )
        hub.SetPos(chrono.ChVectorD(self._wheel_x_cg, y_cg, self._params.wheel.z_cg))
        hub.SetName("Hub")
        return hub

    @cached_property
    def wheel_to_hub_revolute_joint(self) -> chrono.ChLinkLockRevolute:
        """
        Wheel-hub revolute joint.

        Returns
        -------
        chrono.ChLinkLockRevolute

        """
        y_cg = self._wheel_y_cg - self._params.wheel.width / 2
        revolute_csys = chrono.ChCoordsysD(
            chrono.ChVectorD(self._wheel_x_cg, y_cg, self._params.wheel.radius),
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X),
        )
        revolute_joint = chrono.ChLinkLockRevolute()
        revolute_joint.Initialize(self.wheel, self.hub, revolute_csys)
        return revolute_joint

    @cached_property
    def driving_motor(self) -> chrono.ChLinkMotorRotationSpeed:
        """
        Driving motor.

        Returns
        -------
        chrono.ChLinkMotorRotationSpeed

        """
        y_cg = self._wheel_y_cg - self._params.wheel.width / 2
        revolute_csys = chrono.ChFrameD(
            chrono.ChVectorD(self._wheel_x_cg, y_cg, self._params.wheel.radius),
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X),
        )
        motor = chrono.ChLinkMotorRotationSpeed()
        motor.Initialize(self.wheel, self.hub, revolute_csys)
        motor.SetSpeedFunction(self._angular_velocity_function)
        return motor

    @cached_property
    def steering_motor(self) -> chrono.ChLinkMotorRotationAngle:
        """
        Steering motor.

        Returns
        -------
        chrono.ChLinkMotorRotationAngle

        """
        chassis_length = self._params.chassis.length
        chassis_width = self._params.chassis.width
        x_cg = chassis_length / 2 if self._is_front else -chassis_length / 2
        y_cg = chassis_width / 2 if self._is_left else -chassis_width / 2
        revolute_csys = chrono.ChFrameD(
            chrono.ChVectorD(x_cg, y_cg, self._params.chassis.z_cg),
            chrono.ChQuaternionD(1, 0, 0, 0),
        )
        motor = chrono.ChLinkMotorRotationAngle()
        motor.Initialize(self.hub, self._chassis, revolute_csys)
        motor.SetAngleFunction(self._steering_angle_function)
        return motor

    def add_to_system(self, system: chrono.ChSystem) -> None:
        """
        Add components to system.

        Parameters
        ----------
        system : chrono.ChSystem

        """

        system.Add(self.wheel)
        system.Add(self.hub)
        if self._simulation_scenario.are_towing_wheel:
            system.AddLink(self.wheel_to_hub_revolute_joint)
        else:
            system.AddLink(self.driving_motor)
        system.AddLink(self.steering_motor)
