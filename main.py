import math
from typing import List, NamedTuple

from matplotlib import pyplot as plt

mass_earth = 5.972 * (10 ** 24) # kg
radius_earth = 6378000 # m
big_G = 6.6743 * (10 ** (-11)) # m^3 kg^-1 s^-2
ideal_gas_constant = 8.3144598 # N·m/(mol·K)
molar_mass_air = 0.0289644 # kg/mol
tangental_velocity_earth = (radius_earth * 2 * math.pi) / (60 * 60 * 24) # 463.8 m/s
standard_pressure = 101325 # Pascal = N / m^2

def force_gravity(mass: float, altitude: float) -> float:
    # G m M / r ^ 2
    return big_G * mass * mass_earth / (radius_earth + altitude) ** 2

little_g = force_gravity(1.0, 0.0) # 9.8 m/s^2

def force_drag(density: float, velocity: float, drag_coefficient: float, cross_sectional_area: float) -> float:
    # 1/2 rho v^2 c A
    return 0.5 * density * (velocity ** 2) * drag_coefficient * cross_sectional_area


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

class Stage(NamedTuple):
    mass_dry: float # kg
    mass_payload: float # kg
    mass_prop: float # kg
    engines: List[Engine]

def main() -> None:
    #print('acceleration due to gravity', little_g)
    #print('tangental velocity due to earth\'s rotation', tangental_velocity_earth)
    #print('num_engines_exterior', num_engines_exterior(9.0, 1.3))

    # Parameters
    dt = 0.01 # integration timestep, seconds
    acceleration_limit = 4.0 * little_g
    cross_sectional_area = math.pi * 4.5 ** 2 # m^2

    # Initial conditions
    time = 0.0
    altitude = 0.0
    velocity = 0.0
    acceleration = 0.0

    # Accumulate values for plots
    altitudes = [altitude]
    velocities = [velocity]
    accelerations = [0.0]
    barometric_densities = [barometric_density(0.0)]
    dynamic_pressures = [0.0]
    stage_sep_data = []

    superheavy = Stage(200000.0, 0.0, 3400000.0, 33*r2)
    starship = Stage(100000.0, 150000.0, 1200000.0, 3*r2 + 6*r2vac)
    #stages = [superheavy, starship]

    # i.e. 12 rings of turbofans, with the central turbofan + inner 3 rings removed.
    # This should give about 10% more thrust than superheavy, and about 10X more dry mass. (1425 Tons...)
    num_fans = circle_packing_number(12) - circle_packing_number(4) # 360 = 397 - 37
    turbostage = Stage(num_fans*CF6_mass, 0.0, 100000.0, num_fans*turbofan_CF6)
    superheavy_short = Stage(200000.0, 0.0, 3300000.0, 33*r2)
    #stages = [turbostage, superheavy_short, starship]

    # In thrust we trust!
    superheavy_22 = Stage(200000.0, 0.0, 2200000.0, 33*r2 + 25*r2)
    superheavy_12 = Stage(100000.0, 0.0, 1200000.0, 30*r2)
    #stages = [superheavy_22, superheavy_12, starship]
    superheavy_21 = Stage(200000.0, 0.0, 2100000.0, 33*r2 + 25*r2)
    stages = [turbostage, superheavy_21, superheavy_12, starship]

    for i in range(len(stages)):
        # This is the total mass of all the (remaining) stages, excluding the prop mass of the current stage.
        mass_stages = stages[i].mass_dry + stages[i].mass_payload + sum([s.mass_dry + s.mass_payload + s.mass_prop for s in stages[(i+1):]])
        mass_prop_curr = stages[i].mass_prop

        while mass_prop_curr > 0:
            mass_curr = mass_stages + mass_prop_curr

            density = barometric_density(altitude)
            drag_force = force_drag(density, velocity, 0.5, cross_sectional_area)
            thrust = sum([e.dmdt * e.velocity_exhaust for e in stages[i].engines])
            forces_sum = thrust - force_gravity(mass_curr, altitude) - drag_force
            acceleration = forces_sum / mass_curr
        
            excess_thrust_factor = 1.0
            if acceleration > acceleration_limit:
                excess_thrust = (acceleration - acceleration_limit) * mass_curr
                excess_thrust_factor = (thrust - excess_thrust) / thrust
                acceleration = acceleration_limit

            time += dt
            velocity += acceleration * dt
            altitude += velocity * dt

            altitudes += [altitude / 1000.0] # convert to km
            velocities += [velocity]
            accelerations += [acceleration / little_g] # convert to unitless multiples of little_g
            barometric_densities += [density]
            dynamic_pressures += [drag_force / (cross_sectional_area * standard_pressure)] # convert to pressure in units of bar

            mass_prop_curr -= sum([e.dmdt for e in stages[i].engines]) * dt * excess_thrust_factor

        stage_sep_data += [(time, altitude, velocity, accelerations[-1], density, dynamic_pressures[-1])]

    times = [t*dt for t in range(len(altitudes))]
    stage_sep_times = [d[0] for d in stage_sep_data]

    nrows: int = 2
    ncols: int = 3
    #plt.style.use('dark_background')
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(12, 9), sharex=True)
    fig.suptitle('')
    fig.tight_layout()
    plt.subplots_adjust(left=0.08, wspace=0.24, bottom=0.08, hspace=0.24)

    ax = axes[0][0]
    ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
    ax.scatter(times, altitudes, marker='o', s=(72./fig.dpi)**2)  # type: ignore
    ax.set_xlabel('time (seconds)')
    ax.set_ylabel('altitude (km)')
    for sep_time in stage_sep_times:
        ax.vlines(sep_time, min(altitudes), max(altitudes), color='black')

    ax = axes[0][1]
    ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
    ax.scatter(times, velocities, marker='o', s=(72./fig.dpi)**2)  # type: ignore
    ax.set_xlabel('time (seconds)')
    ax.set_ylabel('velocity (m/s)')
    for sep_time in stage_sep_times:
        ax.vlines(sep_time, min(velocities), max(velocities), color='black')

    ax = axes[0][2]
    ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
    ax.scatter(times, accelerations, marker='o', s=(72./fig.dpi)**2)  # type: ignore
    ax.set_xlabel('time (seconds)')
    ax.set_ylabel('acceleration (g)')
    for sep_time in stage_sep_times:
        ax.vlines(sep_time, min(accelerations), max(accelerations), color='black')

    ax = axes[1][0]
    ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
    ax.scatter(times, barometric_densities, marker='o', s=(72./fig.dpi)**2)  # type: ignore
    ax.set_xlabel('time (seconds)')
    ax.set_ylabel('barmoetric density (kg/m^3)')
    for sep_time in stage_sep_times:
        ax.vlines(sep_time, min(barometric_densities), max(barometric_densities), color='black')

    def find_maxQ_time(times: List[float], dynamic_pressures: List[float]) -> float:
        indexed_pressures = list(zip(times, dynamic_pressures))
        indexed_pressures.sort(key=lambda x: x[1])
        return indexed_pressures[-1][0]

    ax = axes[1][1]
    ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')
    ax.scatter(times, dynamic_pressures, marker='o', s=(72./fig.dpi)**2)  # type: ignore
    ax.set_xlabel('time (seconds)')
    ax.set_ylabel('dynamic pressure (bar)')
    for sep_time in stage_sep_times:
        ax.vlines(sep_time, min(dynamic_pressures), max(dynamic_pressures), color='black')
    # Plot maxQ
    maxQ_time = find_maxQ_time(times, dynamic_pressures)
    ax.vlines(maxQ_time, min(dynamic_pressures), max(dynamic_pressures), color='red')
    print('maxQ_time', maxQ_time)

    print('stage_sep_data')
    for d in stage_sep_data:
        print(d)
    print('total dV', velocity)

    plt.show()

if __name__ == '__main__':
    main()
