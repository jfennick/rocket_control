import copy
import math
import random

from matplotlib import pyplot as plt

from . import controls as ctrl
from . import plots
from . import rockets
from . import params
from . import simulation

def main() -> None:
    #print('acceleration due to gravity', little_g)
    #print('tangental velocity due to earth\'s rotation', tangental_velocity_earth)
    #print('num_engines_exterior', num_engines_exterior(9.0, 1.3))

    rockets.starship.controls = ctrl.reduce_times(ctrl.controls_stage2)
    rockets.superheavy.controls = ctrl.reduce_times(ctrl.controls_stage1)
    rockets.superheavy_short.controls = ctrl.reduce_times(ctrl.controls_stage1)
    rockets.turbostage.controls = ctrl.reduce_times(ctrl.controls_stage1)
    rockets.superheavy_22.controls = ctrl.reduce_times(ctrl.controls_stage1a)
    rockets.superheavy_12.controls = ctrl.reduce_times(ctrl.controls_stage1b)
    rockets.superheavy_21.controls = ctrl.reduce_times(ctrl.controls_stage1)
    stages = rockets.stages

    score_best = -math.inf

    controls_copy = [copy.deepcopy(stage.controls) for stage in stages]
    burn_times_copy = [s.burn_time for s in stages]
    max_iters = 1

    nrows: int = len(stages)
    ncols: int = 7
    (fig, axes2d) = plots.initialize_plots(nrows, ncols)

    orbit = False # Has any solution reached orbit yet?
    for i in range(1, 1 + max_iters):
        telemetries = params.get_initial_conditions(stages)

        simulation.run(stages, telemetries)

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
            stage.controls = ctrl.perturb_controls_phi_random(stage.controls, 100, 1)
        # Perturb the boostback burns for the next iteration
        for stage in stages[:-1]: # Exclude the last stage
            delta_time = 1 + (random.random() * 2 - 1) / 1000
            stage.burn_time *= delta_time

    plt.show()

if __name__ == '__main__':
    main()
