mod constants;
mod controls;
mod params;
mod rkt_types;
mod rockets;
mod utils;

//mod rand;

use crate::controls::mcontrols::{up, gravity_turn1, gravity_turn1a, gravity_turn1b,
    staging_delay, boostback, gravity_turn2, over12,
    reduce_times, perturb_controls_phi_random};
use crate::params::mparams::{get_downranges, get_initial_conditions, dt, num_timesteps};
use crate::utils::mutils;
use crate::rkt_types::mrkt_types::{Stage};
use crate::rockets::mrockets::{r2, starship, superheavy, turbostage,
    superheavy_t, superheavy_a, superheavy_b, superheavy_a_t, get_burn_time};


fn main() {
    let (rho, phi) = mutils::cart2pol(3.0, 4.0);
    let (x, y) = mutils::pol2cart(rho, phi);
    println!("{:.64} {:.64}", rho, phi);
    println!("{:.64} {:.64}", x, y);
    let little_g: f64 = mutils::force_gravity(1.0, 0.0);
    //let density = mutils::barometric_density(0.0);
    println!("{:.64}", little_g);

    let telems = get_initial_conditions();
    println!("{:?}", telems[0]);

    let times = (0..num_timesteps).map(|x| dt * (x as f64)).collect::<Vec<_>>();
    let downranges = get_downranges(telems, &times);

    // Our default controls depend on some information in the stages, but the
    // stages also depend on the controls (i.e. the controls are stored in the stages.)
    // There is some slight duplication of engines and prop mass here, but it
    // allows us to define the stages functionally, without mutation.
    let burntime = 0.85 * get_burn_time(3400000.0, & vec![r2; 33]);
    let burntime_a = 0.91 * get_burn_time(2200000.0, & vec![r2; 33 + 25]);
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
    ctrls_2_mut.push(staging_delay);
    ctrls_2_mut.push(over12);
    let ctrls_2 = reduce_times(&ctrls_2_mut);

    //let stages: [Stage; 2] = [superheavy(ctrls_1), starship(ctrls_2)];
    //let stages: [Stage; 3] = [turbostage(ctrls_1), superheavy_t(ctrls_t), starship(ctrls_2)];
    let stages: [Stage; 3] = [superheavy_a(ctrls_1a), superheavy_b(ctrls_1b), starship(ctrls_2)];
    //let stages: [Stage; 4] = [turbostage(ctrls_1), superheavy_a_t(ctrls_a_t), superheavy_b(ctrls_1b), starship(ctrls_2)];
    println!("{:?}", stages);
}
