from __future__ import annotations

import ctypes
from pathlib import Path
from types import SimpleNamespace

import pychrono as chrono
import pychrono.irrlicht as irr

import utils
from plotter import Plotter
from simulation import Simulation
from simulation_scenarios import SimulationScenario, SimulationScenarios
from vehicle import Vehicle

user32 = ctypes.windll.user32


def main(vehicle_params: SimpleNamespace, simulation_params: SimpleNamespace) -> None:
    """
    Entry point.

    Parameters
    ----------
    vehicle_params : SimpleNamespace
    simulation_params : SimpleNamespace

    """
    simulation_scenario = get_simulation_scenario(vehicle_params, simulation_params)
    system = chrono.ChSystemNSC()
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(simulation_scenario.friction)

    ground = utils.create_ground(material, z_offset=0.0)
    vehicle = Vehicle(vehicle_params, material, simulation_scenario)

    system.Set_G_acc(simulation_scenario.gravity_vector)
    system.Add(ground)
    vehicle.add_to_system(system)

    visual_system = create_visual_system(system)

    simulation = Simulation(system, visual_system, vehicle, simulation_scenario)
    if simulation_params.mode == 0:
        view_model(visual_system)
    else:
        simulation.run()
        plotter = Plotter(simulation.results)
        plotter.plot_driving_motor_torques()
        plotter.plot_driving_motor_torques_smoothed()
        plotter.plot_steering_motor_torques()
        plotter.plot_steering_motor_torques_smoothed()


def get_simulation_scenario(
    vehicle_params: SimpleNamespace, simulation_params: SimpleNamespace
) -> SimulationScenario:
    """
    Get simulation scenario.

    Parameters
    ----------
    vehicle_params : SimpleNamespace
    simulation_params : SimpleNamespace

    Returns
    -------
    SimulationScenario

    """
    if simulation_params.scenario_id == 0:
        linear_velocity = simulation_params.linear_velocity
        angular_velocity = utils.convert_linear_to_angular_velocity(
            linear_velocity, vehicle_params.wheel.radius
        )
        angular_velocity_function = utils.compute_ramp_function(
            transient_time=3.0, y_initial=0.0, y_final=angular_velocity
        )
        return SimulationScenario(
            angular_velocity_left_front=angular_velocity_function,
            angular_velocity_right_front=angular_velocity_function,
            angular_velocity_left_rear=angular_velocity_function,
            angular_velocity_right_rear=angular_velocity_function,
            slope=simulation_params.slope,
            friction=simulation_params.friction,
            total_time=simulation_params.total_time,
            time_step=simulation_params.time_step,
        )
    else:
        scenarios = SimulationScenarios(vehicle_params, simulation_params)
        if simulation_params.scenario_id == 1:
            return scenarios.straight_line_constant_speed
        elif simulation_params.scenario_id == 2:
            return scenarios.steering_motors_no_motion


def create_visual_system(system: chrono.ChSystem) -> irr.ChVisualSystemIrrlicht:
    """
    Create visual system.

    Parameters
    ----------
    system : chrono.ChSystem

    Returns
    -------
    irr.ChVisualSystemIrrlicht

    """
    visual_system = irr.ChVisualSystemIrrlicht()
    visual_system.AttachSystem(system)
    visual_system.SetWindowSize(user32.GetSystemMetrics(0), user32.GetSystemMetrics(1))
    visual_system.Initialize()
    visual_system.AddSkyBox()
    visual_system.AddCamera(chrono.ChVectorD(0, 0, 1))
    visual_system.AddTypicalLights()
    visual_system.ShowExplorer(True)
    visual_system.EnableBodyFrameDrawing(True)
    return visual_system


def view_model(visual_system: irr.ChVisualSystemIrrlicht) -> None:
    """
    View model, no simulation.

    Parameters
    ----------
    visual_system : irr.ChVisualSystemIrrlicht

    """
    while visual_system.Run():
        visual_system.BeginScene()
        visual_system.Render()
        visual_system.EndScene()


if __name__ == "__main__":
    vehicle_params_ = utils.convert_mapping_to_namespace(
        utils.to_dict(Path(r"config\vehicle_params.yaml"))
    )
    simulation_params_ = utils.convert_mapping_to_namespace(
        utils.to_dict(Path(r"config\simulation_params.yaml"))
    )
    main(vehicle_params_, simulation_params_)
