from __future__ import annotations

import logging
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List

import numpy as np
import pychrono as chrono
import yaml
from scipy.ndimage import gaussian_filter1d

logger = logging.getLogger(__name__)


def create_ground(
    material: chrono.ChMaterialSurface, z_offset: float = 0.0
) -> chrono.ChBodyEasyBox:
    """
    Create ground.

    Parameters
    ----------
    material : chrono.ChMaterialSurface
    z_offset : float
        The default is 0.0.

    Returns
    -------
    chrono.ChBodyEasyBox

    """
    ground = chrono.ChBodyEasyBox(50.0, 2.0, 0.001, 1000, True, True, material)
    ground.SetPos(chrono.ChVectorD(0.0, 0.0, z_offset))
    ground.SetBodyFixed(True)
    ground.SetName("Ground")
    ground.GetVisualShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/concrete.jpg")
    )
    return ground


def to_dict(path: Path) -> Dict[Any, Any]:
    """
    Import yaml file and convert it to a dictionary.

    Returns
    -------
    Dict[Any, Any]

    """
    try:
        with open(path, "r") as file:
            return yaml.safe_load(file.read())

    except FileNotFoundError:
        logger.error(f"{path} not found.")

    except IOError:
        logger.error(f"{path} import failed.", exc_info=True)


def compute_box_density(
    length: float, width: float, height: float, mass: float
) -> float:
    """
    Compute block density.

    Parameters
    ----------
    length : float
    width : float
    height : float
    mass : float

    Returns
    -------
    float

    """
    return mass / (length * width * height)


def compute_cylinder_density(radius: float, height: float, mass: float) -> float:
    """
    Compute cylinder density.

    Parameters
    ----------
    radius : float
    height : float
    mass : float

    Returns
    -------
    float

    """
    return mass / (np.pi * radius**2 * height)


def convert_mapping_to_namespace(param_to_value: Dict) -> SimpleNamespace:
    """
    Convert a nested dictionary to a namespace.

    Parameters
    ----------
    param_to_value: Dict

    Returns
    -------
    SimpleNamespace

    """
    for key, value in param_to_value.items():
        if isinstance(value, dict):
            param_to_value[key] = convert_mapping_to_namespace(value)
    return SimpleNamespace(**param_to_value)


def convert_linear_to_angular_velocity(
    linear_velocity: float, wheel_radius: float
) -> float:
    """
    Convert linear (km/h) to angular velocity (rad/s).

    Parameters
    ----------
    linear_velocity : float
    wheel_radius : float

    Returns
    -------
    float

    """
    kmh_to_ms = 1 / 3.6
    linear_velocity_ = linear_velocity * kmh_to_ms
    return linear_velocity_ / wheel_radius


def compute_ramp_function(
    transient_time: float, y_initial: float, y_final: float
) -> chrono.ChFunction_Ramp:
    """
    Compute a ramp function.

    Parameters
    ----------
    transient_time : float
    y_initial : float
    y_final : float

    Returns
    -------
    chrono.ChFunction_Ramp

    """
    function = chrono.ChFunction_Ramp()
    function.Set_y0(y_initial)
    function.Set_ang(y_final / transient_time)
    return function


def smooth_data(data: List[float]) -> np.ndarray:
    """
    Smooth data.

    Parameters
    ----------
    data : List[float]

    Returns
    -------
    np.ndarray

    """
    return gaussian_filter1d(data, sigma=5)
