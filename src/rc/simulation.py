import math
import time as timelib
from typing import List

import numpy as np

from .constants import tangental_velocity_earth, radius_earth, standard_pressure, speed_of_sound
from .params import acceleration_limit, dt, num_timesteps, get_downranges, get_initial_conditions, get_stage_time_intervals
from . import rockets
from .rkt_types import Control, Coords, Stage, Telemetry
from .utils import barometric_density, cart2pol, force_drag, force_gravity, pol2cart, magnitude


def get_forces(mass_stages: float, mass_prop_remaining: float,
               i: int, time: float,
               stage: Stage, controls: List[Control],
               position: np.ndarray, velocity: np.ndarray) -> List[float]:

    mass_curr = mass_stages + mass_prop_remaining
    (pos_rho, pos_phi) = cart2pol(position[0], position[1])
    radius = magnitude(position[0], position[1])

    # The velocity of the atmosphere points
    # in the direction of the derivative of position, i.e. + math.pi / 2
    phi = pos_phi + math.pi / 2.0
    (x_comp, y_comp) = pol2cart(1.0, phi)
    atmosphere_vel_phi = tangental_velocity_earth*(pos_rho/radius_earth)
    atm_vel_x = x_comp * atmosphere_vel_phi
    atm_vel_y = y_comp * atmosphere_vel_phi
    # NOTE: The speed used to calculate force_drag should be the NET
    # speed of the stage w.r.t. the atmosphere, NOT just the stage itself!
    # speed_stage = magnitude(velocity[0], velocity[1])
    net_vel_x = velocity[0] - atm_vel_x
    net_vel_y = velocity[1] - atm_vel_y
    speed = magnitude(net_vel_x, net_vel_y)

    altitude = radius - radius_earth

    # Determine forces

    # Force of gravity always points 'down',
    # in the opposite direction to position (in polar coordinates), i.e. - math.pi
    phi = pos_phi - math.pi
    (x_comp, y_comp) = pol2cart(1.0, phi)
    force_gravity_mag = force_gravity(mass_curr, altitude)
    force_gravity_x = x_comp * force_gravity_mag
    force_gravity_y = y_comp * force_gravity_mag

    # Centripetal force always points 'up',
    # in the same direction to position (in polar coordinates)
    # Necessary in cartesian?

    # Force of drag always points
    # in the opposite direction to velocity, i.e. (-1) *
    density = barometric_density(altitude)
    drag_coef = 0.05 if speed < speed_of_sound else 0.15
    force_drag_mag = force_drag(density, speed, drag_coef, rockets.cross_sectional_area)
    #(vel_rho, vel_phi) = cart2pol(velocity[0], velocity[1])
    #force_drag_rho = force_drag_mag * vel_rho / speed
    #force_drag_phi = force_drag_mag * vel_phi / speed
    force_drag_x = 0.0 if speed == 0.0 else (-1.0) * force_drag_mag * net_vel_x / speed
    force_drag_y = 0.0 if speed == 0.0 else (-1.0) * force_drag_mag * net_vel_y / speed

    #forces_sum_rho = thrust_rho - force_gravity(mass_curr, altitude) - force_drag_rho
    #forces_sum_phi = thrust_phi - force_drag_phi
    #(forces_sum_x, forces_sum_y) = pol2cart(forces_sum_rho, forces_sum_phi)
    forces_sum_x = force_gravity_x + force_drag_x
    forces_sum_y = force_gravity_y + force_drag_y

    excess_thrust_factor = 1.0
    if mass_prop_remaining > 0.0:
        # Update the control for the current stage, if necessary
        if not controls[i].t1 <= time < controls[i].t2:
            for c in stage.controls:
                if c.t1 <= time < c.t2:
                    controls[i] = c
                    #print(f'stage {i+1} updating', controls[i])
                    break
                # TODO: Check that there is always a valid control.

        thrust_mag = sum([e.dmdt * e.velocity_exhaust for e in stage.engines])
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

    return [forces_sum_x, forces_sum_y, force_drag_mag, excess_thrust_factor]


def run(stages: List[Stage]) -> List[Telemetry]:
    time_initial = timelib.time()

    # Initialization
    stage_time_intervals = get_stage_time_intervals(stages)
    controls = [stage.controls[0] for stage in stages]
    telemetries = get_initial_conditions(stages)

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

            # copy so we can mutably update @ half-step
            pos0 = np.copy(telemetry.positions[timestep])
            vel0 = np.copy(telemetry.velocities[timestep])

            #print('timestep, stage', timestep, i)
            # This is the total mass of all the (remaining) stages,
            # excluding the remaining prop mass of the current stage.
            mass_stages = stage.mass_dry + stage.mass_payload
            if time < t2:
                mass_stages += sum([s.mass_dry + s.mass_payload + s.mass_prop for s in stages[(i+1):]])
            mass_curr = mass_stages + telemetry.mass_prop_remaining

            forces = get_forces(mass_stages, telemetry.mass_prop_remaining, i, time, stage, controls, pos0, vel0)
            forces_sum_x, forces_sum_y, force_drag_mag, excess_thrust_factor = forces

            prop_used = sum([e.dmdt for e in stage.engines]) * dt * excess_thrust_factor

            # Determine acceleration
            a_x = forces_sum_x / mass_curr
            a_y = forces_sum_y / mass_curr
            acc1 = Coords(a_x, a_y) # using pos0, vel0

            position = np.copy(telemetry.positions[timestep])
            velocity = np.copy(telemetry.velocities[timestep])
            h = dt
            position[0] += h * velocity[0]
            position[1] += h * velocity[1]

            velocity[0] += h * acc1[0]
            velocity[1] += h * acc1[1]

            acceleration = acc1

            midpoint = True
            if midpoint:
                # See https://en.wikipedia.org/wiki/Midpoint_method
                # Now calculate forces again using updated
                # position, velocity, and mass_prop args (and time, if it matters)

                # copy so we can mutably update @ half-step
                pos1 = np.copy(telemetry.positions[timestep])
                vel1 = np.copy(telemetry.velocities[timestep])
                h = dt / 2.0
                pos1[0] += h * vel1[0]
                pos1[1] += h * vel1[1]

                vel1[0] += h * acc1[0]
                vel1[1] += h * acc1[1]

                mass_curr = mass_stages + telemetry.mass_prop_remaining - prop_used / 2.0
                forces = get_forces(mass_stages, telemetry.mass_prop_remaining - prop_used / 2.0,
                                    i, time + h, stage, controls, pos1, vel1)
                forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_ = forces

                # Determine acceleration
                a_x = forces_sum_x / mass_curr
                a_y = forces_sum_y / mass_curr
                acc2 = Coords(a_x, a_y) # using pos1, vel1

                # There are a few timesteps where the two acceleration
                # estimates drastically diverge. This appears to be around
                # stage separation events. For now, simply check for errors
                # and fallback to euler integration.
                acc_mag_ratio = magnitude(acceleration[0], acceleration[1]) / magnitude(acc2[0], acc2[1])
                if abs(acc_mag_ratio - 1.0) < 0.02: # 2% tolerance
                    position = np.copy(telemetry.positions[timestep])
                    velocity = np.copy(telemetry.velocities[timestep])

                    h = dt
                    position[0] += h * velocity[0]
                    position[1] += h * velocity[1]

                    velocity[0] += h * acc2[0]
                    velocity[1] += h * acc2[1]

                    acceleration = acc2
                else:
                    pass #print(i + 1, timestep, round(time, 4), round(acc_mag_ratio, 6))

            runge_kutta_4 = True
            if runge_kutta_4:
                # See https://en.wikipedia.org/wiki/Runge–Kutta_methods
                # Now calculate forces again using updated
                # position, velocity, and mass_prop args (and time, if it matters)

                # copy so we can mutably update @ half-step
                pos2 = np.copy(telemetry.positions[timestep])
                vel2 = np.copy(telemetry.velocities[timestep])

                # Estimate half-timestep velocities, positions using the Euler method.
                h = dt / 2.0
                pos2[0] += h * vel2[0]
                pos2[1] += h * vel2[1]

                vel2[0] += h * acc2[0]
                vel2[1] += h * acc2[1]

                mass_curr = mass_stages + telemetry.mass_prop_remaining - prop_used / 2.0
                forces = get_forces(mass_stages, telemetry.mass_prop_remaining - prop_used / 2.0,
                                    i, time + h, stage, controls, pos2, vel2)
                forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_ = forces

                # Determine acceleration
                a_x = forces_sum_x / mass_curr
                a_y = forces_sum_y / mass_curr
                acc3 = Coords(a_x, a_y) # using pos2, vel2

                # copy so we can mutably update @ next step
                pos3 = np.copy(telemetry.positions[timestep])
                vel3 = np.copy(telemetry.velocities[timestep])

                # Estimate next timestep velocities, positions using the Euler method.
                h = dt
                pos3[0] += h * vel3[0]
                pos3[1] += h * vel3[1]

                vel3[0] += h * acc3[0]
                vel3[1] += h * acc3[1]

                mass_curr = mass_stages + telemetry.mass_prop_remaining - prop_used
                forces = get_forces(mass_stages, telemetry.mass_prop_remaining - prop_used,
                                    i, time + h, stage, controls, pos3, vel3)
                forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_ = forces

                # Determine acceleration
                a_x = forces_sum_x / mass_curr
                a_y = forces_sum_y / mass_curr
                acc4 = Coords(a_x, a_y) # using pos3, vel3

                # Finally, perform a weighted average of the four estimates.
                # This is essentially Simpson's rule.
                vel_rk4_x = (vel0[0] + 2.0*vel1[0] + 2.0*vel2[0] + vel3[0]) / 6.0
                vel_rk4_y = (vel0[1] + 2.0*vel1[1] + 2.0*vel2[1] + vel3[1]) / 6.0
                pos_rk4 = Coords(pos0[0] + dt * vel_rk4_x, pos0[1] + dt * vel_rk4_y)
                acc_rk4_x = (acc1[0] + 2.0*acc2[0] + 2.0*acc3[0] + acc4[0]) / 6.0
                acc_rk4_y = (acc1[1] + 2.0*acc2[1] + 2.0*acc3[1] + acc4[1]) / 6.0
                vel_rk4 = Coords(vel0[0] + dt * acc_rk4_x, vel0[1] + dt * acc_rk4_y)
                acc_rk4 = Coords(acc_rk4_x, acc_rk4_y)

                # There are a few timesteps where the two acceleration
                # estimates drastically diverge. This appears to be around
                # stage separation events. For now, simply check for errors
                # and fallback to euler integration.
                acc_mag_ratio = magnitude(acceleration[0], acceleration[1]) / magnitude(acc_rk4[0], acc_rk4[1])
                if abs(acc_mag_ratio - 1.0) < 0.02: # 2% tolerance
                    position[0] = pos_rk4[0]
                    position[1] = pos_rk4[1]
                    velocity[0] = vel_rk4[0]
                    velocity[1] = vel_rk4[1]
                    acceleration = acc_rk4
                else:
                    pass #print(i + 1, timestep, round(time, 4), round(acc_mag_ratio, 6))

            # NOW mutably update prop after half-timestep
            telemetry.mass_prop_remaining -= prop_used

            telems = [telemetries[i]]
            # Before stage separation i, stages >= i move together, so
            # overwrite the telemetries for stages >= i identically here.
            if time < t2:
                telems = telemetries[i:]

            for telem in telems:
                radius = magnitude(position[0], position[1])
                altitude = radius - radius_earth
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

                    telem.velocities[timestep+1][0] = velocity[0]
                    telem.velocities[timestep+1][1] = velocity[1]

                    telem.positions[timestep+1][0] = position[0]
                    telem.positions[timestep+1][1] = position[1]

                    density = barometric_density(altitude)
                    telem.barometric_densities[timestep+1] = density
                     # convert to pressure in units of bar
                    force_tot_bar = (rockets.cross_sectional_area * standard_pressure)
                    telem.dynamic_pressures[timestep+1] = force_drag_mag / force_tot_bar

    time_final = timelib.time()
    print('simulation time', time_final - time_initial)
    return telemetries

def score_telemetries(orbit: bool, telemetries: List[Telemetry]) -> float:
    # Only care about final stage for orbit check and delta_velocity
    telemetry = telemetries[-1]

    def phis_pos(phi: float) -> float:
        return phi if phi >= 0 else phi + math.pi

    downranges = []
    for telemetry in telemetries:
        phis = [phis_pos(cart2pol(pos[0], pos[1])[1]) for pos in telemetry.positions]
        downranges.append(get_downranges(phis)[-1])

    # NOTE: The various score components will in general have different units.
    # This is okay!
    # NOTE: Use absolute value here instead of squaring the residual; otherwise,
    # the return-to-launch-site penalty can outweigh the dV to orbit!
    # Only care about initial stages for rtls penalty
    rtls_penalty = sum([abs(d) for d in downranges[:-1]])

    # Check if we failed to reach orbit
    if telemetry.positions.size < num_timesteps:
        # If we have not reached orbit yet,
        # we do NOT want discontinuities in the objective function, because
        # that will cause an 'activity cliff' and prevent the algorithm from
        # improving failing solutions. Instead, return something that will
        # monotonically increase as we get closer to reaching orbit, such as
        # the downrange distance of the last stage.
        if orbit:
            return -math.inf
        else:
            # Divide by 100 so this is numerically smaller than the dV
            # required to achieve orbit.
            return downranges[-1] / 100.0

    velocities = [magnitude(vx, vy) for vx, vy in telemetry.velocities]
    delta_velocity = max(velocities)

    score = delta_velocity - rtls_penalty
    return score
