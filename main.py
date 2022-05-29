import copy
import math

import controls as ctrl
import plots
import rockets
import setup
import simulation

def main() -> None:
    #print('acceleration due to gravity', little_g)
    #print('tangental velocity due to earth\'s rotation', tangental_velocity_earth)
    #print('num_engines_exterior', num_engines_exterior(9.0, 1.3))

    rockets.starship.controls = ctrl.controls_stage2
    rockets.superheavy.controls = ctrl.controls_stage1
    rockets.superheavy_short.controls = ctrl.controls_stage1
    rockets.turbostage.controls = ctrl.controls_stage1
    rockets.superheavy_22.controls = ctrl.controls_stage1
    rockets.superheavy_12.controls = ctrl.controls_stage1
    rockets.superheavy_21.controls = ctrl.controls_stage1
    stages = rockets.stages

    stage_time_intervals = setup.get_stage_time_intervals(stages)
    score_best = -math.inf

    controls_copy = [copy.deepcopy(stage.controls) for stage in stages]
    max_perturbs = 100

    for i in range(1, 1 + max_perturbs):
        telemetries = setup.get_initial_conditions(stages)
        simulation.run(stages, telemetries, stage_time_intervals)
        score = simulation.score_telemetries(telemetries)

        # Use a greedy optimiation strategy for now.
        if score > score_best:
            score_best = score
        else:
            # If worse, roll-back to the previous controls
            for stage, control in zip(stages, controls_copy):
                stage.controls = control

        print('score curr', round(score, 3), 'score best', round(score_best, 3))

        if i == max_perturbs:
            break

        controls_copy = [copy.deepcopy(stage.controls) for stage in stages]

        # Perturb the controls for the next iteration
        for stage in stages:
            stage.controls = ctrl.perturb_controls_phi_random(stage.controls, 100)

    stage_sep_times_cumsum = setup.get_stage_sep_times_cumsum(stages)
    plots.make_plots(telemetries, stage_sep_times_cumsum)


if __name__ == '__main__':
    main()
