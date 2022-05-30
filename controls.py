import math
import random
from sys import maxsize
from typing import List

from rkt_types import Control
import rockets

sbt = [s.burn_time for s in rockets.stages]
# Controls (piecewise constant, all times are in seconds)
uptime = 10.0
numturns1 = 10
turntime1 = (sbt[0] - uptime) / numturns1
turntime1a = turntime1
turntime1b = sbt[1] / numturns1
numturns2 = 10
turntime2 = 5.0

# Simplest possible control: straight up, then straight sideways.
up = [Control(0.0, uptime, 0.0)]
over = [Control(uptime, 1000.0, math.pi / 2)]
controls_up_over = up + over

def piecewise_turn(init_time: float, init_angle: float, final_angle: float, numturns: int, turntime: float) -> List[Control]:
    return [Control(init_time + r*turntime, init_time + (r+1)*turntime, init_angle + (final_angle - init_angle)*(r+1)/numturns) for r in range(numturns)]

gravity_turn1 = piecewise_turn(0.0, 0.0, math.pi/4, numturns1, turntime1)
gravity_turn1a = piecewise_turn(0.0, 0.0, math.pi/6, numturns1, turntime1a)
gravity_turn1b = piecewise_turn(0.0, math.pi/6, math.pi/4, numturns1, turntime1b)
diag = [Control(0.0, turntime1b, math.pi / 4, 1.0)]

staging_time = 3.0
staging_delay = [Control(0.0, staging_time, 0.0, 0.0)]

gravity_turn2 = piecewise_turn(0.0, math.pi/4, math.pi/2, numturns2, turntime2)
over12 = [Control(0.0, 1000.0, math.pi / 2)]
boostback = [Control(0.0, 1000.0, - math.pi / 2, 0.4)] # minus = backward, and using 40% thrust

controls_stage1 = up + gravity_turn1 + staging_delay + boostback
controls_stage1a = up + gravity_turn1a + staging_delay + boostback
controls_stage1b = up + gravity_turn1a + staging_delay + gravity_turn1b + staging_delay + boostback
#controls_stage2 = up + gravity_turn1 + staging_delay + gravity_turn2 + over12
controls_stage2 = up + gravity_turn1a + staging_delay + gravity_turn1b + staging_delay + gravity_turn2 + over12

def reduce_times(controls: List[Control]) -> List[Control]:
    time = 0.0
    new_controls = []
    for c in controls:
        delta_t = c.t2 - c.t1
        control = Control(time, time + delta_t, c.force_phi, c.force_mag)
        new_controls.append(control)
        time += delta_t
    return new_controls

def perturb_controls_phi_random(controls: List[Control], scale: int,  index_min: int = 0, index_max: int = maxsize) -> List[Control]:
    new_controls = []
    for i, c in enumerate(controls):
        # Skip controls out of bounds
        if not (index_min <= i < index_max):
            new_control = Control(c.t1, c.t2, c.force_phi, c.force_mag)
        else:
            delta_phi = (random.random() * 2 - 1) * (math.pi / 2) / scale
            new_control = Control(c.t1, c.t2, c.force_phi + delta_phi, c.force_mag)
        new_controls.append(new_control)
    return new_controls