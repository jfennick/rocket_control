import time as timelib
from typing import List

from setup import acceleration_limit, dt, num_timesteps
import rockets
from rkt_types import Coords, Stage, Telemetry
from utils import *

def run(stages: List[Stage], telemetries: List[Telemetry], stage_time_intervals: List[Tuple[float, float]]) -> None:
    time_initial = timelib.time()
    # Initialize controls
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

            # The velocity of the atmosphere points in the direction of the derivative of position, i.e. + math.pi / 2
            phi = pos_phi + math.pi / 2
            (x_comp, y_comp) = pol2cart(1.0, phi)
            atmosphere_vel_phi = tangental_velocity_earth*(pos_rho/radius_earth)
            atm_vel_x = x_comp * atmosphere_vel_phi
            atm_vel_y = y_comp * atmosphere_vel_phi
            velocity = telemetry.velocities[timestep]
            # NOTE: The speed used to calculate force_drag should be the NET
            # speed of the stage w.r.t. the atmosphere, NOT just the stage itself!
            # speed_stage = magnitude(velocity[0], velocity[1])
            net_vel_x = velocity[0] - atm_vel_x
            net_vel_y = velocity[1] - atm_vel_y
            speed = magnitude(net_vel_x, net_vel_y)

            altitude = radius - radius_earth

            # Determine forces

            # Force of gravity always points 'down', in the opposite direction to position (in polar coordinates), i.e. - math.pi
            phi = pos_phi - math.pi
            (x_comp, y_comp) = pol2cart(1.0, phi)
            force_gravity_mag = force_gravity(mass_curr, altitude)
            force_gravity_x = x_comp * force_gravity_mag
            force_gravity_y = y_comp * force_gravity_mag

            # Centripetal force always points 'up', in the same direction to position (in polar coordinates)
            # Necessary in cartesian?

            # Force of drag always points in the opposite direction to velocity, i.e. (-1) *
            density = barometric_density(altitude)
            force_drag_mag = force_drag(density, speed, 0.5, rockets.cross_sectional_area)
            #force_drag_rho = force_drag_mag * vel_rho / speed
            #force_drag_phi = force_drag_mag * vel_phi / speed
            force_drag_x = 0 if speed == 0 else (-1) * force_drag_mag * net_vel_x / speed
            force_drag_y = 0 if speed == 0 else (-1) * force_drag_mag * net_vel_y / speed

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
                            #print(f'stage {i+1} updating', controls[i])
                            break
                        # TODO: Check that there is always a valid control.

                thrust_mag = sum([e.dmdt * e.velocity_exhaust for e in stages[i].engines])
                thrust_mag = thrust_mag * controls[i].force_mag
                # To find the thrust vector, add the position and control angles
                phi = pos_phi + controls[i].force_phi
                (x_comp, y_comp) = pol2cart(1.0, phi)
                thrust_x = x_comp * thrust_mag
                thrust_y = y_comp * thrust_mag

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
            a_x = forces_sum_x / mass_curr
            a_y = forces_sum_y / mass_curr
            acceleration = Coords(a_x, a_y)

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
                    telem.dynamic_pressures[timestep+1] = force_drag_mag / (rockets.cross_sectional_area * standard_pressure) # convert to pressure in units of bar

    time_final = timelib.time()
    #print('simulation time', time_final - time_initial)

def score_telemetries(telemetries: List[Telemetry]) -> float:
    telemetry = telemetries[-1] # Only care about final stage for now

    # Check if we failed to reach orbit
    if telemetry.positions.size < num_timesteps:
        return -math.inf

    #downranges = get_downranges(telemetry)
    velocities = [magnitude(vx, vy) for vx, vy in telemetry.velocities]
    delta_velocity = max(velocities)
    return delta_velocity