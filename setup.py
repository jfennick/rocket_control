from typing import List, Tuple
import numpy as np

from rkt_types import Coords, Stage, Telemetry
from constants import radius_earth, tangental_velocity_earth
from utils import little_g, barometric_density, cart2pol


# Simulation setup
# Parameters
dt = 0.1 # integration timestep, seconds
time_limit = 20000.0 # seconds
num_timesteps = int(time_limit / dt)
times = [t*dt for t in range(num_timesteps)]
acceleration_limit = 4.0 * little_g

# Initial conditions
def get_initial_conditions(stages: List[Stage]) -> List[Telemetry]:
    #time = 0.0
    position_0 = Coords(radius_earth, 0.0)
    velocity_0 = Coords(0.0, tangental_velocity_earth)
    acceleration_0 = Coords(0.0, 0.0)
    telemetries = [Telemetry(num_timesteps, stage.mass_prop, position_0, velocity_0, acceleration_0, barometric_density(0.0), 0.0) for stage in stages]
    return telemetries

def get_stage_sep_times(stages: List[Stage]) -> List[float]: # , staging_delays: List[float]
    stage_burn_times = [s.burn_time for s in stages]
    stage_sep_times = list(np.cumsum([0.0] + stage_burn_times))
    return stage_sep_times

def get_stage_time_intervals(stages: List[Stage]) -> List[Tuple[float, float]]:
    stage_sep_times_cumsum = get_stage_sep_times(stages)
    stage_time_intervals = list(zip(stage_sep_times_cumsum, stage_sep_times_cumsum[1:]))
    return stage_time_intervals

def get_downranges(telemetry: Telemetry) -> List[float]:
    xs = [pos[0] for pos in telemetry.positions]
    ys = [pos[1] for pos in telemetry.positions]
    phis = [cart2pol(pos[0], pos[1])[1] for pos in telemetry.positions]
    downranges = [(radius_earth * phi - tangental_velocity_earth * time) / 1000.0 for time, phi in zip(times, phis)] # Convert to km
    return downranges