import math
import numpy as np
from typing import Tuple

from constants import *

def magnitude(x: float, y: float) -> float:
    return math.sqrt(x ** 2 + y ** 2)

def cart2pol(x: float, y: float) -> Tuple[float, float]:
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return (rho, phi)

def pol2cart(rho: float, phi: float) -> Tuple[float, float]:
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return (x, y)

def force_gravity(mass: float, altitude: float) -> float:
    # G m M / r ^ 2
    return big_G * mass * mass_earth / (radius_earth + altitude) ** 2

little_g = force_gravity(1.0, 0.0) # 9.8 m/s^2

def force_drag(density: float, speed: float, drag_coefficient: float, cross_sectional_area: float) -> float:
    # 1/2 rho v^2 c A
    return 0.5 * density * (speed ** 2) * drag_coefficient * cross_sectional_area
    
def barometric_density(altitude: float) -> float:
    # See https://en.wikipedia.org/wiki/Barometric_formula#Density_equations
    # altitude (m), density (kg/m^3), temperature (K), temperature lapse rate (K/m)
    piecewise_data = [(0.00000, 1.22500, 288.15, -0.0065),
                      (11000.0, 0.36391, 216.65, 0.0),
                      (20000.0, 0.08803, 216.65, 0.001),
                      (32000.0, 0.01322, 228.65, 0.0028),
                      (47000.0, 0.00143, 270.65, 0.0),
                      (51000.0, 0.00086, 270.65, -0.0028),
                      (71000.0, 0.000064, 214.65, -0.002),
                      (84852.0, 0.00000, 000.00, 0.0)] # only altitude used here
    for d, d2 in zip(piecewise_data, piecewise_data[1:]):
        altitude_b: float = d[0]
        altitude_b2: float = d2[0]
        if altitude >= altitude_b and altitude < altitude_b2:
            rho_b: float = d[1]
            temp_b: float = d[2]
            lapse_b: float = d[3]
            if lapse_b == 0.0:
                numerator = (-1.0) * little_g * molar_mass_air * (altitude - altitude_b)
                denominator = ideal_gas_constant * temp_b
                return rho_b * math.exp(numerator / denominator)
            else:
                base: float = temp_b / (temp_b + (altitude - altitude_b) * lapse_b)
                exponent: float = 1.0 + (little_g * molar_mass_air) / (ideal_gas_constant * lapse_b)
                return rho_b * base ** exponent # type: ignore
    # else, we are in space
    return 0.0
