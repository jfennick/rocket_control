import copy
import math
import random

from matplotlib import pyplot as plt

from .controls import (up, gravity_turn1, gravity_turn1a, gravity_turn1b,
                      staging_delay, boostback, gravity_turn2, over12,
                      reduce_times, perturb_controls_phi_random)
from . import plots
from .rockets import (r2, starship, superheavy, turbostage, superheavy_t,
                     superheavy_a, superheavy_b, superheavy_a_t, get_burn_time)

from . import params
from . import simulation

import rocket_control as rc # Rust bindings (generated from `maturin develop`)

def main() -> None:
    #print('acceleration due to gravity', little_g)
    #print('tangental velocity due to earth\'s rotation', tangental_velocity_earth)
    #print('num_engines_exterior', num_engines_exterior(9.0, 1.3))

    # Our default controls depend on some information in the stages, but the
    # stages also depend on the controls (i.e. the controls are stored in the stages.)
    # There is some slight duplication of engines and prop mass here, but it
    # allows us to define the stages functionally, without mutation.
    burntime = 0.85 * get_burn_time(3400000.0, 33*r2)
    burntime_a = 0.91 * get_burn_time(2200000.0, 33*r2 + 25*r2)
    burntime_b = 0.77 * get_burn_time(1200000.0, 30*r2)
    burntime_t = 0.85 * get_burn_time(3400000.0 - 50000.0, 33*r2)
    burntime_a_t = 0.91 * get_burn_time(2200000.0 - 50000.0, 33*r2 + 25*r2)

    ctrls_1 = reduce_times(up + gravity_turn1(burntime) + staging_delay + boostback)
    ctrls_1a = reduce_times(up + gravity_turn1a(burntime_a) + staging_delay + boostback)
    ctrls_1b = reduce_times(up + gravity_turn1a(burntime_a) + staging_delay + gravity_turn1b(burntime_b) + staging_delay + boostback)
    # ctrls_t = ...
    # ctrls_a_t = ...

    #ctrls_2 = reduce_times(up + gravity_turn1(burntime1) + staging_delay + gravity_turn2 + over12)
    ctrls_2 = reduce_times(up + gravity_turn1a(burntime_a) + staging_delay + gravity_turn1b(burntime_b) + staging_delay + gravity_turn2 + over12)

    #stages = [superheavy(ctrls_1), starship(ctrls_2)]
    #stages = [turbostage(ctrls_1), superheavy_t(ctrls_t), starship(ctrls_2)]
    stages = [superheavy_a(ctrls_1a), superheavy_b(ctrls_1b), starship(ctrls_2)]
    #stages = [turbostage(ctrls_1), superheavy_a_t(ctrls_a_t), superheavy_b(ctls_b), starship(ctrls_2)]

    score_best = -math.inf

    controls_copy = [copy.deepcopy(stage.controls) for stage in stages]
    burn_times_copy = [s.burn_time for s in stages]
    max_iters = 1

    nrows: int = len(stages)
    ncols: int = 7
    (fig, axes2d) = plots.initialize_plots(nrows, ncols)

    (result, controls) = rc.main()
    posx = [[t.position.x for t in r] for r in result]
    posy = [[t.position.y for t in r] for r in result]
    velx = [[t.velocity.x for t in r] for r in result]
    vely = [[t.velocity.y for t in r] for r in result]
    accx = [[t.acceleration.x for t in r] for r in result]
    accy = [[t.acceleration.y for t in r] for r in result]
    baro = [[t.barometric_density for t in r] for r in result]
    pres = [[t.dynamic_pressure for t in r] for r in result]
    stage_sep_times = params.get_stage_sep_times(stages)
    plots.update_plots(fig, axes2d, controls, posx, posy, velx, vely, accx, accy, baro, pres, stage_sep_times)
    #plt.show()
    plt.savefig('Plots.png')
    import sys
    sys.exit(0)
    # TODO: Either finish synchronizing the python and rust code (i.e. for performance comparison)
    # or eliminate the python simulation code and just use python for plotting.

    orbit = False # Has any solution reached orbit yet?
    for i in range(1, 1 + max_iters):
        telemetries = simulation.run(stages)

        score = simulation.score_telemetries(orbit, telemetries)

        # Did we just reach orbit?
        orbit_i = not telemetries[-1].positions.size < params.num_timesteps
        if orbit_i:
            orbit = True

        # Use a greedy optimization strategy for now.
        if score > score_best:
            score_best = score

            stage_sep_times = params.get_stage_sep_times(stages)
            plots.update_plots(fig, axes2d, controls_copy, telemetries, stage_sep_times)
            # NOTE: Do NOT use time.sleep(1.0) here!
            # It does NOT restart the GUI event loop!
            plots.pause_no_show(0.1)  # Wait at least 0.1 second so we don't just spin.
        else:
            # If worse, roll-back to the previous controls
            for stage, control in zip(stages, controls_copy):
                stage.controls = control
            for stage, burn_time in zip(stages, burn_times_copy):
                stage.burn_time = burn_time

        print('iter', i, 'score curr', round(score, 3), 'score best', round(score_best, 3))

        if i == max_iters:
            break

        controls_copy = [copy.deepcopy(stage.controls) for stage in stages]
        burn_times_copy = [s.burn_time for s in stages]

        # Perturb the controls for the next iteration
        for stage in stages:
            stage.controls = perturb_controls_phi_random(stage.controls, 100.0, 1)
        # Perturb the boostback burns for the next iteration
        for stage in stages[:-1]: # Exclude the last stage
            delta_time = 1.0 + (random.random() * 2.0 - 1.0) / 1000.0
            stage.burn_time *= delta_time

    plt.show()

if __name__ == '__main__':
    main()
