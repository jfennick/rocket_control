import math

from rkt_types import Engine, Stage
from utils import little_g

# CF6-50 Turbofan
CF6_mass = 3960 # kg
CF6_TWR =  6.01 # unitless
CF6_SFC =  0.0105 # (kg/s)/kN
CF6_thrust = (CF6_mass * little_g) * CF6_TWR # Newton
CF6_fuel_rate = (CF6_thrust / 1000.0) * CF6_SFC
#print('CF6_thrust (kN)', CF6_thrust)
#print('CF6_thrust (Tons)', CF6_thrust / (little_g * 1000.0))
#print('CF6_fuel_rate (kg/s)', CF6_fuel_rate)
turbofan_CF6 = [Engine(CF6_fuel_rate, 95000, 2.19)]

def num_engines_exterior(diameter_rocket: float, diameter_engine: float) -> int:
    # Close enough for a first approximation.
    return round(math.pi*(diameter_rocket - diameter_engine)/diameter_engine)

def triangular_number(n: int) -> int:
    return int(n * (n + 1) / 2)

def circle_packing_number(n: int) -> int:
    return 1 + 6 * triangular_number(n - 1)

#print('circle_packing_numbers 1 to 12')
#print([circle_packing_number(n) for n in range(1, 14)])


# Rocket / staging setup
cross_sectional_area = math.pi * 4.5 ** 2 # m^2

r2 = [Engine(650.0, 3200.0, 1.3)]
r2vac = [Engine(650.0, 3560.0, 2.4)]

superheavy = Stage(200000.0, 0.0, 3400000.0, 33*r2)
superheavy.burn_time *= 0.85 # Save for boostback burn
starship = Stage(100000.0, 120000.0, 1200000.0, 3*r2 + 6*r2vac)
#stages = [superheavy, starship]

# i.e. 12 hexagonal rings of turbofans, with the central turbofan + inner 3 rings removed.
# This should give about 10% more thrust than superheavy, and about 5X more dry mass. (1425 Tons...)
# Note that this dry mass estimate does not include any of the structural mass
# necessary to actually connect the turbofans together and to the rocket, but
# it also doesn't include any mass savings you would get by designing an
# optimized turbfon specifically for this application.
num_fans = circle_packing_number(12) - circle_packing_number(4) # 360 = 397 - 37
turbostage = Stage(num_fans*CF6_mass, 0.0, 50000.0, num_fans*turbofan_CF6)
superheavy_short = Stage(200000.0, 0.0, superheavy.mass_prop - turbostage.mass_prop, 33*r2)
#stages = [turbostage, superheavy_short, starship]

# In thrust we trust!
superheavy_22 = Stage(200000.0, 0.0, 2200000.0, 33*r2 + 25*r2)
superheavy_22.burn_time *= 0.91 # Save for boostback burn
superheavy_12 = Stage(100000.0, 0.0, 1200000.0, 30*r2)
superheavy_12.burn_time *= 0.77 # Save for boostback burn
stages = [superheavy_22, superheavy_12, starship]
superheavy_21 = Stage(200000.0, 0.0, superheavy_22.mass_prop - turbostage.mass_prop, 33*r2 + 25*r2)
#stages = [turbostage, superheavy_21, superheavy_12, starship]