import math

mass_earth = 5.972 * (10 ** 24) # kg
radius_earth = 6378000 # m
big_G = 6.6743 * (10 ** (-11)) # m^3 kg^-1 s^-2
# little_g in utils.py
ideal_gas_constant = 8.3144598 # N·m/(mol·K)
molar_mass_air = 0.0289644 # kg/mol
tangental_velocity_earth = (radius_earth * 2 * math.pi) / (60 * 60 * 24) # 463.8 m/s
standard_pressure = 101325 # Pascal = N / m^2
speed_of_sound = 343.0 # m/s

#velocity_escape = math.sqrt(2 * big_G * mass_earth / radius_earth)
