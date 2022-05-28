import math
import time as timelib
from typing import List, NamedTuple, Tuple

from matplotlib import pyplot as plt
import numpy as np

mass_earth = 5.972 * (10 ** 24) # kg
radius_earth = 6378000 # m
big_G = 6.6743 * (10 ** (-11)) # m^3 kg^-1 s^-2
ideal_gas_constant = 8.3144598 # N·m/(mol·K)
molar_mass_air = 0.0289644 # kg/mol
tangental_velocity_earth = (radius_earth * 2 * math.pi) / (60 * 60 * 24) # 463.8 m/s
standard_pressure = 101325 # Pascal = N / m^2

velocity_escape = math.sqrt(2 * big_G * mass_earth / radius_earth)

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


def num_engines_exterior(diameter_rocket: float, diameter_engine: float) -> int:
    # Close enough for a first approximation.
    return round(math.pi*(diameter_rocket - diameter_engine)/diameter_engine)

def triangular_number(n: int) -> int:
    return int(n * (n + 1) / 2)

def circle_packing_number(n: int) -> int:
    return 1 + 6 * triangular_number(n - 1)

#print('circle_packing_numbers 1 to 12')
#print([circle_packing_number(n) for n in range(1, 14)])
    
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

class Engine(NamedTuple):
    dmdt            : float # kg/s
    velocity_exhaust: float # m/s, in rocket frame
    diameter        : float # m

r2 = [Engine(650.0, 3200.0, 1.3)]
r2vac = [Engine(650.0, 3560.0, 2.4)]

# CF6-50
CF6_mass = 3960 # kg
CF6_TWR =  6.01 # unitless
CF6_SFC =  0.0105 # (kg/s)/kN
CF6_thrust = (CF6_mass * little_g) * CF6_TWR # Newton
CF6_fuel_rate = (CF6_thrust / 1000.0) * CF6_SFC
#print('CF6_thrust (kN)', CF6_thrust)
#print('CF6_thrust (Tons)', CF6_thrust / (little_g * 1000.0))
#print('CF6_fuel_rate (kg/s)', CF6_fuel_rate)
turbofan_CF6 = [Engine(CF6_fuel_rate, 95000, 2.19)]

class Coords(NamedTuple):
    x: float
    y: float

def magnitude(x: float, y: float) -> float:
    return math.sqrt(x ** 2 + y ** 2)

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
                 controls: List[Control] = []) -> None:
        self.mass_dry = mass_dry
        self.mass_payload = mass_payload
        self.mass_prop = mass_prop
        self.engines = engines
        self.controls = controls
        
    mass_dry: float # kg
    mass_payload: float # kg
    mass_prop: float # kg
    engines: List[Engine]
    controls: List[Control] = []


def main() -> None:
    #print('acceleration due to gravity', little_g)
    #print('tangental velocity due to earth\'s rotation', tangental_velocity_earth)
    #print('num_engines_exterior', num_engines_exterior(9.0, 1.3))

    # Rocket / staging setup
    cross_sectional_area = math.pi * 4.5 ** 2 # m^2

    superheavy = Stage(200000.0, 0.0, 3400000.0, 33*r2)
    starship = Stage(100000.0, 120000.0, 1200000.0, 3*r2 + 6*r2vac)
    stages = [superheavy, starship]

    # i.e. 12 hexagonal rings of turbofans, with the central turbofan + inner 3 rings removed.
    # This should give about 10% more thrust than superheavy, and about 10X more dry mass. (1425 Tons...)
    num_fans = circle_packing_number(12) - circle_packing_number(4) # 360 = 397 - 37
    turbostage = Stage(num_fans*CF6_mass, 0.0, 50000.0, num_fans*turbofan_CF6)
    superheavy_short = Stage(200000.0, 0.0, superheavy.mass_prop - turbostage.mass_prop, 33*r2)
    #stages = [turbostage, superheavy_short, starship]

    # In thrust we trust!
    superheavy_22 = Stage(200000.0, 0.0, 2200000.0, 33*r2 + 25*r2)
    superheavy_12 = Stage(100000.0, 0.0, 1200000.0, 30*r2)
    #stages = [superheavy_22, superheavy_12, starship]
    superheavy_21 = Stage(200000.0, 0.0, superheavy_22.mass_prop - turbostage.mass_prop, 33*r2 + 25*r2)
    #stages = [turbostage, superheavy_21, superheavy_12, starship]

    # Simulation setup
    # Parameters
    dt = 0.1 # integration timestep, seconds
    time_limit = 10000.0 # seconds
    num_timesteps = int(time_limit / dt)
    acceleration_limit = 4.0 * little_g

    # Initial conditions
    #time = 0.0
    position_0 = Coords(radius_earth, 0.0) # start 1.0m to the east :)
    velocity_0 = Coords(0.0, tangental_velocity_earth)
    acceleration_0 = Coords(0.0, 0.0)
    telemetries = [Telemetry(num_timesteps, stage.mass_prop, position_0, velocity_0, acceleration_0, barometric_density(0.0), 0.0) for stage in stages]

    stage_sep_times = [stage.mass_prop / sum([e.dmdt for e in stage.engines]) for stage in stages]
    stage_sep_times = [0.85 * stage_sep_times[0], stage_sep_times[1]]
    # TODO: consider acceleration limit (burnout times will be longer than the above full-throttle estimate)
    #stage_sep_times[-1] = time_limit
    stage_sep_times_cumsum = np.cumsum([0.0] + stage_sep_times)
    stage_time_intervals = list(zip(stage_sep_times_cumsum, stage_sep_times_cumsum[1:]))

    # Controls (piecewise constant, all times are in seconds)
    uptime = 10.0
    numturns1 = 10
    turntime1 = (stage_sep_times[0] - uptime) / numturns1
    numturns2 = 10
    turntime2 = 10.0

    # Simplest possible control: straight up, then straight sideways.
    up = [Control(0.0, uptime, 0.0)]
    over = [Control(uptime, 1000.0, math.pi / 2)]
    controls_up_over = up + over

    def piecewise_turn(init_time: float, init_angle: float, final_angle: float, numturns: int, turntime: float) -> List[Control]:
        return [Control(init_time + r*turntime, init_time + (r+1)*turntime, init_angle + (final_angle - init_angle)*(r+1)/numturns) for r in range(numturns)]

    gravity_turn1 = piecewise_turn(uptime, 0.0, math.pi/4, numturns1, turntime1)
    staging_time = 5.0
    staging_delay = [Control(uptime + numturns1*turntime1, uptime + numturns1*turntime1 + staging_time, 0.0, 0.0)]
    gravity_turn2 = piecewise_turn(uptime + numturns1*turntime1 + staging_time, math.pi/4, math.pi/2, numturns2, turntime2)
    over12 = [Control(uptime + numturns1*turntime1 + staging_time + numturns2*turntime2, 1000.0, math.pi / 2)]
    boostback = [Control(uptime + numturns1*turntime1 + staging_time, 1000.0, - math.pi / 2, 0.4)] # minus = backward, and using 40% thrust
    controls_stage1 = up + gravity_turn1 + staging_delay + boostback
    controls_stage2 = up + gravity_turn1 + staging_delay + gravity_turn2 + over12

    starship.controls = controls_stage2
    superheavy.controls = controls_stage1
    superheavy_short.controls = controls_stage1
    turbostage.controls = controls_stage1
    superheavy_22.controls = controls_stage1
    superheavy_12.controls = controls_stage1
    superheavy_21.controls = controls_stage1

    time_initial = timelib.time()

    controls = [stage.controls[0] for stage in stages]

    for timestep in range(num_timesteps - 1):
        time = timestep * dt
        for i, stage in enumerate(stages):
            (t1, t2) = stage_time_intervals[i]
            # Before stage separation i, stages >= i move together, so only
            # calculate forces and accelerations for the combined stages once
            # and overwrite the telemetries for stages >= i identically below.
            if time < t1:
                continue

            telemetry = telemetries[i]
            if not telemetry.update:
                continue

            #print('timestep, stage', timestep, i)
            # This is the total mass of all the (remaining) stages, excluding the remaining prop mass of the current stage.
            mass_stages = stage.mass_dry + stage.mass_payload
            if time < t2:
                mass_stages += sum([s.mass_dry + s.mass_payload + s.mass_prop for s in stages[(i+1):]])
            mass_curr = mass_stages + telemetry.mass_prop_remaining

            # Get telemetry from previous timestep
            position = telemetry.positions[timestep]
            (pos_rho, pos_phi) = cart2pol(position[0], position[1])
            radius = magnitude(position[0], position[1])

            velocity = telemetry.velocities[timestep]
            (vel_rho, vel_phi) = cart2pol(velocity[0], velocity[1])
            speed = magnitude(velocity[0], velocity[1])

            altitude = radius - radius_earth

            # Determine forces

            # Force of gravity always points 'down', in the opposite direction to position (in polar coordinates), i.e. - math.pi
            force_gravity_mag = force_gravity(mass_curr, altitude)
            phi = pos_phi - math.pi
            (fx_comp, fy_comp) = pol2cart(1.0, phi)
            force_gravity_x = fx_comp * force_gravity_mag
            force_gravity_y = fy_comp * force_gravity_mag

            # Centripetal force always points 'up', in the same direction to position (in polar coordinates)
            # Necessary in cartesian?

            # Force of drag always points in the opposite direction to velocity, i.e. (-1) *
            density = barometric_density(altitude)
            force_drag_mag = force_drag(density, speed, 0.5, cross_sectional_area)
            #force_drag_rho = force_drag_mag * vel_rho / speed
            #force_drag_phi = force_drag_mag * vel_phi / speed
            force_drag_x = 0.0 #(-1) * force_drag_mag * velocity[0] / speed
            force_drag_y = 0.0 #(-1) * force_drag_mag * velocity[1] / speed

            #forces_sum_rho = thrust_rho - force_gravity(mass_curr, altitude) - force_drag_rho
            #forces_sum_phi = thrust_phi - force_drag_phi
            #(forces_sum_x, forces_sum_y) = pol2cart(forces_sum_rho, forces_sum_phi)
            forces_sum_x = force_gravity_x + force_drag_x
            forces_sum_y = force_gravity_y + force_drag_y

            if telemetry.mass_prop_remaining > 0:
                # Update the control for the current stage, if necessary
                if not (controls[i].t1 <= time < controls[i].t2):
                    for c in stage.controls:
                        if c.t1 <= time < c.t2:
                            controls[i] = c
                            print(f'stage {i+1} updating', controls[i])
                            break
                        # TODO: Check that there is always a valid control.

                thrust_mag = sum([e.dmdt * e.velocity_exhaust for e in stages[i].engines])
                thrust_mag = thrust_mag * controls[i].force_mag
                # To find the thrust vector, add the position and control angles
                phi = pos_phi + controls[i].force_phi
                (fx_comp, fy_comp) = pol2cart(1.0, phi)
                thrust_x = fx_comp * thrust_mag
                thrust_y = fy_comp * thrust_mag

                #forces_sum_mag = magnitude(forces_sum_rho, forces_sum_phi)
                # Include all forces, or just thrust? Let's just use thrust.
                # For the purpose of limiting structural loads we can ignore gravity,
                # and the acceleration near maxQ is nowhere near this limit.
                acceleration_mag = thrust_mag / mass_curr

                excess_thrust_factor = 1.0
                if acceleration_mag > acceleration_limit:
                    excess_thrust = (acceleration_mag - acceleration_limit) * mass_curr
                    excess_thrust_factor = (thrust_mag - excess_thrust) / thrust_mag
                    thrust_x = thrust_x * acceleration_limit / acceleration_mag
                    thrust_y = thrust_y * acceleration_limit / acceleration_mag

                forces_sum_x += thrust_x
                forces_sum_y += thrust_y
                telemetry.mass_prop_remaining -= sum([e.dmdt for e in stage.engines]) * dt * excess_thrust_factor

            # Determine acceleration
            ax = forces_sum_x / mass_curr
            ay = forces_sum_y / mass_curr
            acceleration = Coords(ax, ay)

            telems = [telemetries[i]]
            # Before stage separation i, stages >= i move together, so
            # overwrite the telemetries for stages >= i identically here.
            if time < t2:
                telems = telemetries[i:]

            for telem in telems:
                if not telem.update:
                    pass
                elif altitude < 0.0:
                    # We either landed or crashed into the dirt.
                    telem.accelerations = telem.accelerations[:timestep]
                    telem.velocities = telem.velocities[:timestep]
                    telem.positions = telem.positions[:timestep]
                    telem.barometric_densities = telem.barometric_densities[:timestep]
                    telem.dynamic_pressures = telem.dynamic_pressures[:timestep]
                    telem.update = False
                else:
                    # Integrate trajectory / update telemetry
                    telem.accelerations[timestep][0] = acceleration[0]
                    telem.accelerations[timestep][1] = acceleration[1]

                    telem.velocities[timestep+1][0] = velocity[0] + acceleration[0]*dt
                    telem.velocities[timestep+1][1] = velocity[1] + acceleration[1]*dt

                    telem.positions[timestep+1][0] = position[0] + velocity[0]*dt
                    telem.positions[timestep+1][1] = position[1] + velocity[1]*dt

                    telem.barometric_densities[timestep+1] = density
                    telem.dynamic_pressures[timestep+1] = force_drag_mag / (cross_sectional_area * standard_pressure) # convert to pressure in units of bar

    time_final = timelib.time()
    print('simulation time', time_final - time_initial)

    nrows: int = len(stages)
    ncols: int = 7
    #plt.style.use('dark_background')
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(18, 9), sharex=False) # True
    fig.suptitle('')
    fig.tight_layout()
    plt.subplots_adjust(left=0.08, wspace=0.24, bottom=0.08, hspace=0.24)

    def stride(nums, max_nums = 1000):
        skip = int(len(nums) / max_nums)
        return [num for i, num in enumerate(nums) if i % skip == 0]

    times = [t*dt for t in range(num_timesteps)]

    for i, telemetry in enumerate(telemetries):
        pos_polar = [cart2pol(x, y) for x, y in telemetry.positions]
        altitudes = [(rho - radius_earth) / 1000.0 for rho, phi in pos_polar] # Convert to km
        times_alt = times[:len(altitudes)]
        ax = axes[i][0]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        ax.scatter(stride(times_alt), stride(altitudes), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('altitude (km)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(altitudes), max(altitudes), color='black')

        velocities = [magnitude(vx, vy) for vx, vy in telemetry.velocities]
        times_vel = times[:len(velocities)]
        ax = axes[i][1]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        ax.scatter(stride(times_vel), stride(velocities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('velocity (m/s)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(velocities), max(velocities), color='black')

        accelerations = [magnitude(ax, ay) / little_g for ax, ay in telemetry.accelerations] # Convert to multiple of little_g
        times_acc = times[:len(accelerations)]
        ax = axes[i][2]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        ax.scatter(stride(times_acc), stride(accelerations), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('acceleration (g)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(accelerations), max(accelerations), color='black')

        densities = list(telemetry.barometric_densities)
        times_den = times[:len(densities)]
        ax = axes[i][3]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        ax.scatter(stride(times_den), stride(densities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('density (kg/m^3)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(telemetry.barometric_densities), max(telemetry.barometric_densities), color='black')

        def find_maxQ_time(times: List[float], dynamic_pressures: List[float]) -> float:
            indexed_pressures = list(zip(times, dynamic_pressures))
            indexed_pressures.sort(key=lambda x: x[1])
            return indexed_pressures[-1][0]

        pressures = list(telemetry.dynamic_pressures)
        times_pre = times[:len(pressures)]
        ax = axes[i][4]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        ax.scatter(stride(times_pre), stride(pressures), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('dynamic pressure (bar)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(pressures), max(pressures), color='black')
        # Plot maxQ
        maxQ_time = find_maxQ_time(times_pre, pressures)
        ax.vlines(maxQ_time, min(telemetry.dynamic_pressures), max(telemetry.dynamic_pressures), color='red')

        ax = axes[i][5]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        xs = [pos[0] for pos in telemetry.positions]
        ys = [pos[1] for pos in telemetry.positions]
        ax.scatter(stride(xs), stride(ys), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('x (meters)')
        ax.set_ylabel('y (meters)')

        ax = axes[i][6]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
        xs = [pos[0] for pos in telemetry.positions]
        ys = [pos[1] for pos in telemetry.positions]
        phis = [cart2pol(pos[0], pos[1])[1] for pos in telemetry.positions]
        downranges = [(radius_earth * phi - tangental_velocity_earth * time) / 1000.0 for time, phi in zip(times, phis)] # Convert to km
        ax.scatter(stride(downranges), stride(altitudes), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('downrange (km)')
        ax.set_ylabel('altitude (km)')

        print(f'stage {i+1} sep data')
        time = stage_sep_times_cumsum[1:][i]
        timestep = min(int(time / dt), num_timesteps - 1)
        timestep = min(timestep, len(altitudes) - 1)
        print('time', time)
        print('maxQ_time', maxQ_time)
        print('altitude', altitudes[timestep])
        print('velocity', velocities[timestep])
        print('acceleration', accelerations[timestep])
        print('density', telemetry.barometric_densities[timestep])
        print('pressure', telemetry.dynamic_pressures[timestep])
        print()

    plt.show()

if __name__ == '__main__':
    main()
