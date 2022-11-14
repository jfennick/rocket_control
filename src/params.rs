pub mod mparams {
    use crate::constants::mconstants::{radius_earth, tangental_velocity_earth};
    use crate::rkt_types::mrkt_types::{Coords, Stage, Telemetry};
    use crate::utils::mutils::{barometric_density, cart2pol, little_g};

    // Simulation setup
    // Parameters
    pub const dt: f64 = 0.1; // integration timestep, seconds
    pub const time_limit: f64 = 10000.0; // seconds
    pub const num_timesteps: usize = (time_limit / dt) as usize;
    pub const acceleration_limit: f64 = 4.0 * little_g;

    pub fn get_initial_conditions() -> Vec<Telemetry> {
        let telem = Telemetry {
            position: Coords { x: 0.0, y: 0.0 },
            velocity: Coords { x: 0.0, y: 0.0 },
            acceleration: Coords { x: 0.0, y: 0.0 },
            barometric_density: 0.0,
            dynamic_pressure: 0.0,
        };
        let mut telems: Vec<Telemetry> = vec![telem; num_timesteps];

        telems[0].position.x = radius_earth;
        telems[0].velocity.y = tangental_velocity_earth;
        telems[0].barometric_density = barometric_density(0.0);
        telems
    }

    pub fn get_stage_sep_times(stages: &Vec<Stage>) -> Vec<f64> {
        // , staging_delays: List[float]
        let stage_burn_times = stages.iter().map(|s| s.burn_time).collect::<Vec<_>>();
        let stage_sep_times = stage_burn_times
            .iter()
            .scan(0.0, |tot, x| {*tot += x; Some(*tot)})
            .collect();
        stage_sep_times
    }

    pub fn get_stage_time_intervals(stages: &Vec<Stage>) -> Vec<(f64, f64)> {
        let stage_sep_times = get_stage_sep_times(stages);
        let stage_sep_times_tail = &stage_sep_times[1..]; // 1.. causes index out of bounds
        let zipped = stage_sep_times.iter().zip(stage_sep_times_tail.iter());
        let stage_time_intervals: Vec<(f64, f64)> = zipped.map(|(f1, f2)| (*f1, *f2)).collect();
        let mut stage_time_interval = vec![(0.0, stage_sep_times[0])];
        stage_time_interval.extend(stage_time_intervals);
        stage_time_interval
    }

    pub fn get_downranges(telems: & Vec<Telemetry>, times: &Vec<f64>) -> Vec<f64> {
        let phis: Vec<f64> = telems
            .iter()
            .map(|t| cart2pol(t.position.x, t.position.y).1)
            .collect();
        let zipped = times.iter().zip(phis.iter());
        let downranges = zipped
            .map(|(time, phi)| (radius_earth * phi - tangental_velocity_earth * time) / 1000.0)
            .collect();
        downranges
    }
}
