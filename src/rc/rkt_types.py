from typing import List, NamedTuple
import numpy as np


class Engine(NamedTuple):
    dmdt            : float # kg/s
    velocity_exhaust: float # m/s, in rocket frame
    diameter        : float # m
    mass            : float # kg

class Coords(NamedTuple):
    x: float
    y: float

# Cannot use NamedTuple because Tuples are immutable
class Telemetry:
    def __init__(self,
                 num_timesteps: int,
                 mass_prop: float,
                 position: Coords,
                 velocity: Coords,
                 acceleration: Coords,
                 barometric_density: float,
                 dynamic_pressure: float) -> None:
        num_coords = len(position)
        self.positions = np.zeros(shape=(num_timesteps, num_coords))
        self.velocities = np.zeros(shape=(num_timesteps, num_coords))
        self.accelerations = np.zeros(shape=(num_timesteps, num_coords))
        self.barometric_densities = np.zeros(shape=(num_timesteps))
        self.dynamic_pressures = np.zeros(shape=(num_timesteps))

        self.mass_prop_remaining = mass_prop
        self.positions[0] = position
        self.velocities[0] = velocity
        self.accelerations[0] = acceleration
        self.barometric_densities[0] = barometric_density
        self.dynamic_pressures[0] = dynamic_pressure
        self.update = True

    mass_prop_remaining: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    barometric_densities: np.ndarray
    dynamic_pressures: np.ndarray
    update: bool

class Control(NamedTuple):
    # t1 < t2
    t1: float
    t2: float
    # In polar coordinates, this is the angle measured from vertical / radial
    force_phi: float
    force_mag: float = 1.0

class Stage():
    def __init__(self,
                 mass_dry: float,
                 mass_payload: float,
                 mass_prop: float,
                 engines: List[Engine],
                 burn_time: float,
                 controls: List[Control]) -> None:
        self.mass_dry = mass_dry
        self.mass_payload = mass_payload
        self.mass_prop = mass_prop
        self.engines = engines
        self.burn_time = burn_time
        self.controls = controls

    #mass_dry: float # kg
    #mass_payload: float # kg
    #mass_prop: float # kg
    #engines: List[Engine]
    #burn_time: float # sec
    #controls: List[Control]
