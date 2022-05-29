from typing import List

from matplotlib import pyplot as plt

from constants import radius_earth
from rkt_types import Telemetry
from rockets import stages
from setup import times, get_downranges
from utils import magnitude, cart2pol, little_g


def make_plots(telemetries: List[Telemetry], stage_sep_times_cumsum: List[float]) -> None:
    nrows: int = len(stages)
    ncols: int = 7
    #plt.style.use('dark_background')
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(18, 9), sharex=False) # True
    fig.suptitle('')
    fig.tight_layout()
    plt.subplots_adjust(left=0.08, wspace=0.24, bottom=0.08, hspace=0.24)

    def stride(nums: List[float], max_nums: int = 1000) -> List[float]:
        skip = int(len(nums) / max_nums)
        return [num for i, num in enumerate(nums) if i % skip == 0]

    for i, telemetry in enumerate(telemetries):
        pos_polar = [cart2pol(x, y) for x, y in telemetry.positions]
        altitudes = [(rho - radius_earth) / 1000.0 for rho, phi in pos_polar] # Convert to km
        times_alt = times[:len(altitudes)]
        ax = axes[i][0]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_alt), stride(altitudes), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('altitude (km)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(altitudes), max(altitudes), color='black')  # type: ignore

        velocities = [magnitude(vx, vy) for vx, vy in telemetry.velocities]
        times_vel = times[:len(velocities)]
        ax = axes[i][1]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_vel), stride(velocities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('velocity (m/s)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(velocities), max(velocities), color='black')  # type: ignore

        accelerations = [magnitude(ax, ay) / little_g for ax, ay in telemetry.accelerations] # Convert to multiple of little_g
        times_acc = times[:len(accelerations)]
        ax = axes[i][2]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_acc), stride(accelerations), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('acceleration (g)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(accelerations), max(accelerations), color='black')  # type: ignore

        densities = list(telemetry.barometric_densities)
        times_den = times[:len(densities)]
        ax = axes[i][3]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_den), stride(densities), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('density (kg/m^3)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(telemetry.barometric_densities), max(telemetry.barometric_densities), color='black')  # type: ignore

        def find_maxQ_time(times: List[float], dynamic_pressures: List[float]) -> float:
            indexed_pressures = list(zip(times, dynamic_pressures))
            indexed_pressures.sort(key=lambda x: x[1])
            return indexed_pressures[-1][0]

        pressures = list(telemetry.dynamic_pressures)
        times_pre = times[:len(pressures)]
        ax = axes[i][4]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        ax.scatter(stride(times_pre), stride(pressures), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('dynamic pressure (bar)')
        for sep_time in stage_sep_times_cumsum[1:]:
            ax.vlines(sep_time, min(pressures), max(pressures), color='black')  # type: ignore
        # Plot maxQ
        maxQ_time = find_maxQ_time(times_pre, pressures)
        ax.vlines(maxQ_time, min(telemetry.dynamic_pressures), max(telemetry.dynamic_pressures), color='red')  # type: ignore

        ax = axes[i][5]
        ax.ticklabel_format(style='sci', scilimits=(-2,3), axis='both')  # type: ignore
        xs = [pos[0] for pos in telemetry.positions]
        ys = [pos[1] for pos in telemetry.positions]
        ax.scatter(stride(xs), stride(ys), marker='o', s=(72./fig.dpi)**2)  # type: ignore
        ax.set_xlabel('x (meters)')
        ax.set_ylabel('y (meters)')

        ax = axes[i][6]
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

    plt.show()