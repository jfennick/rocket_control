import math
import time
from typing import List, Tuple

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import _pylab_helpers # type: ignore

from .constants import radius_earth
from .rkt_types import Control, Telemetry
from .params import dt, times, get_downranges
from .utils import magnitude, cart2pol, little_g


def pause_no_show(interval: float) -> None:
    """This is verbatim copy and pasted from matplotlib.pyplot.pause(), with show() commented out.
    show() rudely brings the window to the foreground, which interrupts the user.

    Args:
        interval (float): Delay execution for a given number of seconds.
        The argument may be a floating point number for subsecond precision.
    """
    manager = _pylab_helpers.Gcf.get_active()
    if manager is not None:
        canvas = manager.canvas
        if canvas.figure.stale:
            canvas.draw_idle()
        #show(block=False)
        canvas.start_event_loop(interval)
    else:
        time.sleep(interval)


def initialize_plots(nrows: int, ncols: int) -> Tuple[matplotlib.pyplot.Figure, List[List[matplotlib.pyplot.Axes]]]:
    """Initialize a grid of blank subplots.

    Args:
        nrows (int): the number of rows
        ncols (int): the number of columns

    Returns:
        Tuple[matplotlib.pyplot.Figure, List[List[matplotlib.pyplot.Axes]]]: The main figure and subplots axes
    """
    #plt.style.use('dark_background') # type: ignore
    fig, axes2d = plt.subplots(nrows=nrows, ncols=ncols, figsize=(18, 9.5))
    # The purpose of calling suptitle is to take up some blank space. suptitle
    # displays a title above all of the subplots, but NOT for the window itself.
    fig.suptitle('')
    plt.get_current_fig_manager().set_window_title('Real-time Analysis Plots') # type: ignore
    fig.tight_layout()
    plt.subplots_adjust(left=0.05, wspace=0.24, bottom=0.05, hspace=0.24)
    plt.show(block=False) # type: ignore
    return (fig, axes2d)


def update_plots(fig: matplotlib.pyplot.Figure, axes2d: List[List[matplotlib.pyplot.Axes]],
                 controls: List[List[Control]],
                 telemetries: List[Telemetry], stage_sep_times: List[float]) -> None:
    """Update the previously initialized plots with the data from telemetries

    Args:
        fig (matplotlib.pyplot.Figure): The main figure
        axes2d (List[List[matplotlib.pyplot.Axes]]): The subplots axes
    """
    nrows = len(axes2d)
    ncols = len(axes2d[0])

    for axes in axes2d:
        for ax in axes:
            ax.clear() # type: ignore

    def stride(nums: List[float], max_nums: int = 1000) -> List[float]:
        skip = int(len(nums) / max_nums)
        return nums if skip == 0 else [num for i, num in enumerate(nums) if i % skip == 0]

    for i, telemetry in enumerate(telemetries):
        pos_polar = [cart2pol(x, y) for x, y in telemetry.positions]
        altitudes = [(rho - radius_earth) / 1000.0 for rho, phi in pos_polar] # Convert to km
        times_alt = times[:len(altitudes)]
        ax = axes2d[i][0]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_alt), stride(altitudes), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('altitude (km)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(altitudes), max(altitudes), color='black')  # type: ignore

        velocities = [magnitude(vx, vy) for vx, vy in telemetry.velocities]
        times_vel = times[:len(velocities)]
        ax = axes2d[i][1]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_vel), stride(velocities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('velocity (m/s)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(velocities), max(velocities), color='black')  # type: ignore

        # Convert to multiple of little_g
        accelerations = [magnitude(ax, ay) / little_g for ax, ay in telemetry.accelerations]
        times_acc = times[:len(accelerations)]
        ax = axes2d[i][2]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_acc), stride(accelerations), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('acceleration (g)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(accelerations), max(accelerations), color='black')  # type: ignore

        """densities = list(telemetry.barometric_densities)
        times_den = times[:len(densities)]
        ax = axes2d[i][3]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_den), stride(densities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('density (kg/m^3)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(telemetry.barometric_densities), max(telemetry.barometric_densities),
                      color='black')  # type: ignore"""

        control_phis = []
        for c in controls[i]:
            num_steps_c = int((c.t2 - c.t1) / dt)
            phi_deg = c.force_phi * (180 / math.pi)
            for j in range(num_steps_c):
                control_phis.append(phi_deg)
        control_phis = control_phis[:telemetry.positions.size]
        times_den = times[:len(control_phis)]
        ax = axes2d[i][3]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_den), stride(control_phis), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('control phi angle (degrees)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(control_phis), max(control_phis), color='black')  # type: ignore

        def find_maxq_time(times_: List[float], dynamic_pressures: List[float]) -> float:
            indexed_pressures = list(zip(times_, dynamic_pressures))
            indexed_pressures.sort(key=lambda x: x[1])
            return indexed_pressures[-1][0]

        pressures = list(telemetry.dynamic_pressures)
        times_pre = times[:len(pressures)]
        ax = axes2d[i][4]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_pre), stride(pressures), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('dynamic pressure (bar)')
        for sep_time in stage_sep_times[1:]:
            ax.vlines(sep_time, min(pressures), max(pressures), color='black')  # type: ignore
        # Plot maxQ
        maxq_time = find_maxq_time(times_pre, pressures)
        ax.vlines(maxq_time, min(telemetry.dynamic_pressures), max(telemetry.dynamic_pressures),
                  color='red')  # type: ignore

        ax = axes2d[i][5]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        xs = [pos[0] for pos in telemetry.positions]
        ys = [pos[1] for pos in telemetry.positions]
        ax.scatter(stride(xs), stride(ys), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('x (meters)')
        ax.set_ylabel('y (meters)')

        ax = axes2d[i][6]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        downranges = get_downranges(telemetry)
        ax.scatter(stride(downranges), stride(altitudes), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('downrange (km)')
        ax.set_ylabel('altitude (km)')

        """print(f'stage {i+1} sep data')
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
        print()"""
