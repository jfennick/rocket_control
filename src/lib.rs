use pyo3::prelude::*;
use pyo3::types::PyTuple;

mod constants;
mod controls;
mod params;
mod rkt_types;
mod rockets;
mod simulation;
mod utils;

//mod rand;
use rand::prelude::*;

use crate::controls::mcontrols::{up, gravity_turn1, gravity_turn1a, gravity_turn1b,
    staging_delay, boostback, gravity_turn2, over12,
    reduce_times, perturb_controls_phi_random, perturb_controls_dt_random};
use crate::params::mparams::{dt, num_timesteps};
use crate::rkt_types::mrkt_types::{Coords, Control, Stage, Telemetry};
use crate::rockets::mrockets::{r2, starship, superheavy, turbostage,
    superheavy_t, superheavy_a, superheavy_b, superheavy_a_t, get_burn_time};
use crate::simulation::msimulation::{run, score_telemetries};

#[pyfunction]
fn main() -> PyResult<(Vec<Vec<Telemetry>>, Vec<Vec<Control>>)> {
    let times = (0..num_timesteps).map(|x| dt * (x as f64)).collect::<Vec<_>>();

    // Our default controls depend on some information in the stages, but the
    // stages also depend on the controls (i.e. the controls are stored in the stages.)
    // There is some slight duplication of engines and prop mass here, but it
    // allows us to define the stages functionally, without mutation.
    let burntime = 0.85 * get_burn_time(3400000.0, & vec![r2; 33]);
    let burntime_a = 0.95 * get_burn_time(2200000.0, & vec![r2; 33 + 25]);
    let burntime_b = 0.77 * get_burn_time(1200000.0, & vec![r2; 30]);
    let burntime_t = 0.85 * get_burn_time(3400000.0 - 50000.0, & vec![r2; 33]);
    let burntime_a_t = 0.91 * get_burn_time(2200000.0 - 50000.0, & vec![r2; 33 + 25]);

    let ctrls_1_mut = &mut vec![];
    ctrls_1_mut.push(up);
    ctrls_1_mut.append(&mut gravity_turn1(burntime));
    ctrls_1_mut.push(staging_delay);
    ctrls_1_mut.push(boostback);
    let ctrls_1 = reduce_times(&ctrls_1_mut);

    let ctrls_1a_mut = &mut vec![];
    ctrls_1a_mut.push(up);
    ctrls_1a_mut.append(&mut gravity_turn1a(burntime_a));
    ctrls_1a_mut.push(staging_delay);
    ctrls_1a_mut.push(boostback);
    let ctrls_1a = reduce_times(&ctrls_1a_mut);

    let ctrls_1b_mut = &mut vec![];
    ctrls_1b_mut.push(up);
    ctrls_1b_mut.append(&mut gravity_turn1a(burntime_a));
    ctrls_1b_mut.push(staging_delay);
    ctrls_1b_mut.append(&mut gravity_turn1b(burntime_b));
    ctrls_1b_mut.push(staging_delay);
    ctrls_1b_mut.push(boostback);
    let ctrls_1b = reduce_times(&ctrls_1b_mut);
    // let ctrls_t = ...
    // let ctrls_a_t = ...

    //let ctrls_2 = reduce_times(up + gravity_turn1(burntime1) + staging_delay + gravity_turn2 + over12)

    let ctrls_2_mut = &mut vec![];
    ctrls_2_mut.push(up);
    ctrls_2_mut.append(&mut gravity_turn1a(burntime_a));
    ctrls_2_mut.push(staging_delay);
    ctrls_2_mut.append(&mut gravity_turn1b(burntime_b));
    ctrls_2_mut.push(staging_delay);
    ctrls_2_mut.append(&mut gravity_turn2());
    //ctrls_2_mut.push(staging_delay);
    ctrls_2_mut.push(over12);
    let ctrls_2 = reduce_times(&ctrls_2_mut);

    //let stages = vec![superheavy(ctrls_1), starship(ctrls_2)];
    //let stages = vec![turbostage(ctrls_1), superheavy_t(ctrls_t), starship(ctrls_2)];
    let stages = vec![superheavy_a(&ctrls_1a), superheavy_b(&ctrls_1b), starship(&ctrls_2)];
    //let stages = vec![turbostage(ctrls_1), superheavy_a_t(ctrls_a_t), superheavy_b(ctrls_1b), starship(ctrls_2)];
    //println!("{:?}", stages);

    let mut score_best = -f64::INFINITY;
    let max_iters = 1000;

    let mut orbit = false; // Has any solution reached orbit yet?
    let controls: &mut Vec<Control> = &mut stages.iter().map(|s| s.controls[0].clone()).collect();
    let mut controls_all: &mut Vec<Vec<Control>>  = &mut stages.iter().map(|s| s.controls.clone()).collect();
    let mut controls_all_copy = &mut controls_all.clone();
    let mut scale_factor = 10.0;
    let mut short_circuit = true; // ~4X performance improvement

    for i in 0..max_iters {
        if i == max_iters - 1 {
            short_circuit = false; // On the last iteration, run the entire simulation for plotting purposes.
        }
        let (telemetries, mass_prop_remaining) = run(&stages, controls, controls_all, short_circuit);
        if i == 1000 {
            // reset score_best when we change score functions
            //score_best = -f64::INFINITY;
        }
        let score = score_telemetries(orbit, &telemetries, &times, &mass_prop_remaining, i);

        // Did we just reach orbit?
        let telemetry = &telemetries[telemetries.len()-1];
        let orbit_i = if telemetry[telemetry.len()-1].position.x == 0.0 {false} else {true};
        if orbit_i {
            orbit = true;
        }

        // Use a greedy optimization strategy for now.
        if score > score_best {
            score_best = score;
            println!("iter {:?} score curr {:?} score best {:?}", i, (score * 1000.0).round() / 1000.0, (score_best * 1000.0).round() / 1000.0);
            println!("mass_prop_remaining {:?}", mass_prop_remaining);
        } else {
            // If worse, roll-back to the previous controls
            for idx in 0..controls.len() {
                controls_all[idx] = controls_all_copy[idx].clone();
            }
            // TODO: roll-back burn times
        }

        if i == max_iters - 1 {
            //break;
            return Ok((telemetries, controls_all.to_vec()));
        }

        for idx in 0..controls.len() {
            controls_all_copy[idx] = controls_all[idx].clone();
        }
        let burn_times_copy: Vec<f64> = stages.iter().map(|s| s.burn_time).collect();

        // Perturb the controls for the next iteration
        for j in 0..controls_all.len()-1 {
            let mut rng = rand::thread_rng();
            let frac: f64 = rng.gen(); // generates a float between 0 and 1
            let index = (frac * controls_all[j].len() as f64) as i32;
            if i < 1000 {
                scale_factor = 10.0; // 100.0
                controls_all[j] = perturb_controls_dt_random(&controls_all[j], scale_factor, 1, controls_all[j].len() as i32 - 1); //mutate all indices
            } else {
                scale_factor = 100.0; // 100.0
                controls_all[j] = perturb_controls_dt_random(&controls_all[j], scale_factor, index, index + 1); //mutate a single index
            }
            //println!("{:?}", controls_all[j])
        }

        // Perturb the boostback burns for the next iteration
        for stage in stages[0..stages.len()-1].to_vec() { // Exclude the last stage
            let mut rng = rand::thread_rng();
            let y: f64 = rng.gen(); // generates a float between 0 and 1
            let delta_time = 1.0 + (y * 2.0 - 1.0) / 1000.0;
            //stage.burn_time *= delta_time
        }
    }
    Ok((vec![], vec![]))
}

/// A Python module implemented in Rust.
#[pymodule]
fn rocket_control(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(main, m)?)?;
    //m.add_class::<Coords>()?;
    Ok(())
}