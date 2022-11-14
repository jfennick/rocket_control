pub mod msimulation {
    use std::f64::consts::PI;

    use crate::constants::mconstants::{tangental_velocity_earth, radius_earth, standard_pressure, speed_of_sound};
    use crate::params::mparams::{acceleration_limit, dt, num_timesteps, get_downranges, get_initial_conditions, get_stage_time_intervals};
    use crate::rockets::mrockets;
    use crate::rkt_types::mrkt_types::{Control, Coords, Stage, Telemetry};
    use crate::utils::mutils::{barometric_density, cart2pol, force_drag, force_gravity, pol2cart, magnitude};

    pub fn get_forces(mass_stages: f64, mass_prop_remaining: f64, i: usize, time: f64, stage: & Stage, controls: &mut Vec<Control>, controls_all: &mut Vec<Vec<Control>>,
        pos_x: f64, pos_y: f64, vel_x: f64, vel_y: f64) -> [f64; 4] {
        let mass_curr = mass_stages + mass_prop_remaining;
        let (pos_rho, pos_phi) = cart2pol(pos_x, pos_y);
        let radius = magnitude(pos_x, pos_y);

        // The velocity of the atmosphere points
        // in the direction of the derivative of position, i.e. + PI / 2
        let phi = pos_phi + PI / 2.0;
        let (x_comp, y_comp) = pol2cart(1.0, phi);
        let atmosphere_vel_phi = tangental_velocity_earth*(pos_rho/radius_earth);
        let atm_vel_x = x_comp * atmosphere_vel_phi;
        let atm_vel_y = y_comp * atmosphere_vel_phi;
        // NOTE: The speed used to calculate force_drag should be the NET
        // speed of the stage w.r.t. the atmosphere, NOT just the stage itself!
        // speed_stage = magnitude(vel_x, vel_y);
        let net_vel_x = vel_x - atm_vel_x;
        let net_vel_y = vel_y - atm_vel_y;
        let speed = magnitude(net_vel_x, net_vel_y);

        let altitude = radius - radius_earth;

        // Determine forces

        // Force of gravity always points 'down',
        // in the opposite direction to position (in polar coordinates), i.e. - math.pi
        let phi = pos_phi - PI;
        let (x_comp, y_comp) = pol2cart(1.0, phi);
        let force_gravity_mag = force_gravity(mass_curr, altitude);
        let force_gravity_x = x_comp * force_gravity_mag;
        let force_gravity_y = y_comp * force_gravity_mag;

        // Centripetal force always points 'up',
        // in the same direction to position (in polar coordinates)
        // Necessary in cartesian?

        // Force of drag always points
        // in the opposite direction to velocity, i.e. (-1) *
        let density = barometric_density(altitude);
        let drag_coef = if speed < speed_of_sound {0.05} else {0.15};
        let force_drag_mag = force_drag(density, speed, drag_coef, mrockets::cross_sectional_area);
        //let (vel_rho, vel_phi) = cart2pol(vel_x, vel_y);
        //let force_drag_rho = force_drag_mag * vel_rho / speed;
        //let force_drag_phi = force_drag_mag * vel_phi / speed;
        let force_drag_x = if speed == 0.0  {0.0} else {(-1.0) * force_drag_mag * net_vel_x / speed};
        let force_drag_y = if speed == 0.0 {0.0} else {(-1.0) * force_drag_mag * net_vel_y / speed};
        
        //let forces_sum_rho = thrust_rho - force_gravity(mass_curr, altitude) - force_drag_rho;
        //let forces_sum_phi = thrust_phi - force_drag_phi;
        //let (forces_sum_x, forces_sum_y) = pol2cart(forces_sum_rho, forces_sum_phi);
        let mut forces_sum_x = force_gravity_x + force_drag_x;
        let mut forces_sum_y = force_gravity_y + force_drag_y;

        let mut excess_thrust_factor = 1.0;

        if mass_prop_remaining > 0.0 {
            // Update the control for the current stage, if necessary
            if !(controls[i].t1 <= time && time < controls[i].t2) {
                for c in &controls_all[i] {
                    if c.t1 <= time && time < c.t2 {
                        controls[i] = c.clone();
                        //println!("stage {:?} updating {:?}", i+1, controls[i]);
                        break;
                    }
                    // TODO: Check that there is always a valid control.
                }
            }

            let thrust_mag: f64 = stage.engines.iter().map(|e| e.dmdt * e.velocity_exhaust).sum();
            let thrust_mag = thrust_mag * controls[i].force_mag;
            // To find the thrust vector, add the position and control angles
            let phi = pos_phi + controls[i].force_phi;
            let (x_comp, y_comp) = pol2cart(1.0, phi);
            let mut thrust_x = x_comp * thrust_mag;
            let mut thrust_y = y_comp * thrust_mag;

            //let forces_sum_mag = magnitude(forces_sum_rho, forces_sum_phi);
            // Include all forces, or just thrust? Let's just use thrust.
            // For the purpose of limiting structural loads we can ignore gravity,
            // and the acceleration near maxQ is nowhere near this limit.
            let acceleration_mag = thrust_mag / mass_curr;

            //excess_thrust_factor = 1.0;
            if acceleration_mag > acceleration_limit {
                let excess_thrust = (acceleration_mag - acceleration_limit) * mass_curr;
                excess_thrust_factor = (thrust_mag - excess_thrust) / thrust_mag;
                thrust_x = thrust_x * acceleration_limit / acceleration_mag;
                thrust_y = thrust_y * acceleration_limit / acceleration_mag;
            }
            forces_sum_x += thrust_x;
            forces_sum_y += thrust_y;
        }
        return [forces_sum_x, forces_sum_y, force_drag_mag, excess_thrust_factor];
    }

    pub fn run(stages: &Vec<Stage>, controls: &mut Vec<Control>, controls_all: &mut Vec<Vec<Control>>) -> Vec<Vec<Telemetry>> {
        let now = std::time::Instant::now();

        // Initialization
        let stage_time_intervals = get_stage_time_intervals(stages);

        let mut mass_prop_remaining: Vec<f64> = stages.iter().map(|s| s.mass_prop).collect();
        let mut update_telemetry: Vec<bool> = stages.iter().map(|s| true).collect();
        let mut telemetries: &mut Vec<Vec<Telemetry>> = &mut stages.iter().map(|s| get_initial_conditions()).collect();

        for timestep in 0..(num_timesteps - 1) {
            let time = (timestep as f64) * dt;
            //println!("time {:?}", time);
            for (i, stage) in stages.iter().enumerate() {
                let (t1, t2) = stage_time_intervals[i];
                // Before stage separation i, stages >= i move together, so only
                // calculate forces and accelerations for the combined stages once
                // and overwrite the telemetries for stages >= i identically below.
                if time < t1 {
                    //println!("{:?} {:?}", time, t1);
                    continue
                }

                let telemetry = &telemetries[i];
                if !update_telemetry[i] {
                    continue
                }

                // copy so we can mutably update @ half-step
                let pos0 = telemetry[timestep].position.clone();
                let vel0 = telemetry[timestep].velocity.clone();

                //println!("timestep {:?} stage {:?}", timestep, i);
                // This is the total mass of all the (remaining) stages,
                // excluding the remaining prop mass of the current stage.
                let mut mass_stages = stage.mass_dry + stage.mass_payload;
                if time < t2 {
                    mass_stages += stages[i+1..].iter().map(|s| s.mass_dry + s.mass_payload + s.mass_prop).sum::<f64>();
                }
                let mass_curr = mass_stages + mass_prop_remaining[i];

                let forces = get_forces(mass_stages, mass_prop_remaining[i], i, time, stage, controls, controls_all, pos0.x, pos0.y, vel0.x, vel0.y);
                let [forces_sum_x, forces_sum_y, force_drag_mag, excess_thrust_factor] = forces;

                let prop_used = stage.engines.iter().map(|e| e.dmdt).sum::<f64>() * dt * excess_thrust_factor;

                // Determine acceleration
                let a_x = forces_sum_x / mass_curr;
                let a_y = forces_sum_y / mass_curr;
                let acc1 = Coords{x: a_x, y: a_y}; // using pos0, vel0
    
                let mut position = telemetry[timestep].position.clone();
                let mut velocity = telemetry[timestep].velocity.clone();

                let h = dt;
                position.x += h * velocity.x;
                position.y += h * velocity.y;
    
                velocity.x += h * acc1.x;
                velocity.y += h * acc1.y;
    
                let mut acceleration = acc1.clone();

                let mut vel1 = Coords{x: 0.0, y: 0.0}; // For scoping reasons
                let mut acc2 = Coords{x: 0.0, y: 0.0}; // For scoping reasons

                let midpoint = true;
                if midpoint {
                    // See https://en.wikipedia.org/wiki/Midpoint_method
                    // Now calculate forces again using updated
                    // position, velocity, and mass_prop args (and time, if it matters)
    
                    // copy so we can mutably update @ half-step
                    let mut pos1 = telemetry[timestep].position.clone();
                    vel1 = telemetry[timestep].velocity.clone();
                    let h = dt / 2.0;
                    pos1.x += h * vel1.x;
                    pos1.x += h * vel1.y;
    
                    vel1.x += h * acc1.x;
                    vel1.y += h * acc1.y;
    
                    let mass_curr = mass_stages + mass_prop_remaining[i] - prop_used / 2.0;
                    let forces = get_forces(mass_stages, mass_prop_remaining[i] - prop_used / 2.0,
                                           i, time + h, stage, controls, controls_all, pos1.x, pos1.y, vel1.x, vel1.y);
                    let [forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_] = forces;
    
                    // Determine acceleration
                    let a_x = forces_sum_x / mass_curr;
                    let a_y = forces_sum_y / mass_curr;
                    acc2 = Coords{x: a_x, y: a_y}; // using pos1, vel1
    
                    // There are a few timesteps where the two acceleration
                    // estimates drastically diverge. This appears to be around
                    // stage separation events. For now, simply check for errors
                    // and fallback to euler integration.
                    let acc_mag_ratio = magnitude(acceleration.x, acceleration.y) / magnitude(acc2.x, acc2.y);
                    if (acc_mag_ratio - 1.0).abs() < 0.02 { // 2% tolerance
                        position = telemetry[timestep].position.clone();
                        velocity = telemetry[timestep].velocity.clone();
    
                        let h = dt;
                        position.x += h * velocity.x;
                        position.y += h * velocity.y;
    
                        velocity.x += h * acc2.x;
                        velocity.y += h * acc2.y;
    
                        acceleration = acc2.clone();
                    } else {
                        //println!("{:?} {:?} {:?} {:?}", i + 1, timestep, (time * 10000.0).round() / 10000.0, (acc_mag_ratio * 1000000.0).round() / 1000000.0);
                    }
                }

                let runge_kutta_4 = true;
                if runge_kutta_4 {
                    // See https://en.wikipedia.org/wiki/Runge–Kutta_methods
                    // Now calculate forces again using updated
                    // position, velocity, and mass_prop args (and time, if it matters)
    
                    // copy so we can mutably update @ half-step
                    let mut pos2 = telemetry[timestep].position.clone();
                    let mut vel2 = telemetry[timestep].velocity.clone();
    
                    // Estimate half-timestep velocities, positions using the Euler method.
                    let h = dt / 2.0;
                    pos2.x += h * vel2.x;
                    pos2.y += h * vel2.y;
    
                    vel2.x += h * acc2.x;
                    vel2.y += h * acc2.y;
    
                    let mass_curr = mass_stages + mass_prop_remaining[i] - prop_used / 2.0;
                    let forces = get_forces(mass_stages, mass_prop_remaining[i] - prop_used / 2.0,
                                           i, time + h, stage, controls, controls_all, pos2.x, pos2.y, vel2.x, vel2.y);
                    let [forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_] = forces;
    
                    // Determine acceleration
                    let a_x = forces_sum_x / mass_curr;
                    let a_y = forces_sum_y / mass_curr;
                    let acc3 = Coords{x: a_x, y: a_y}; // using pos2, vel2
    
                    // copy so we can mutably update @ next step
                    let mut pos3 = telemetry[timestep].position.clone();
                    let mut vel3 = telemetry[timestep].velocity.clone();
    
                    // Estimate next timestep velocities, positions using the Euler method.
                    let h = dt;
                    pos3.x += h * vel3.x;
                    pos3.y += h * vel3.y;
    
                    vel3.x += h * acc3.x;
                    vel3.y += h * acc3.y;
    
                    let mass_curr = mass_stages + mass_prop_remaining[i] - prop_used;
                    let forces = get_forces(mass_stages, mass_prop_remaining[i] - prop_used,
                                           i, time + h, stage, controls, controls_all, pos3.x, pos3.y, vel3.x, vel3.y);
                    let [forces_sum_x, forces_sum_y, force_drag_mag_, excess_thrust_factor_] = forces;
    
                    // Determine acceleration
                    let a_x = forces_sum_x / mass_curr;
                    let a_y = forces_sum_y / mass_curr;
                    let acc4 = Coords{x: a_x, y: a_y}; // using pos3, vel3
    
                    // Finally, perform a weighted average of the four estimates.
                    // This is essentially Simpson's rule.
                    let vel_rk4_x = (vel0.x + 2.0*vel1.x + 2.0*vel2.x + vel3.x) / 6.0;
                    let vel_rk4_y = (vel0.y + 2.0*vel1.y + 2.0*vel2.y + vel3.y) / 6.0;
                    let pos_rk4 = Coords{x: pos0.x + dt * vel_rk4_x, y: pos0.y + dt * vel_rk4_y};
                    let acc_rk4_x = (acc1.x + 2.0*acc2.x + 2.0*acc3.x + acc4.x) / 6.0;
                    let acc_rk4_y = (acc1.y + 2.0*acc2.y + 2.0*acc3.y + acc4.y) / 6.0;
                    let vel_rk4 = Coords{x: vel0.x + dt * acc_rk4_x, y: vel0.y + dt * acc_rk4_y};
                    let acc_rk4 = Coords{x: acc_rk4_x, y: acc_rk4_y};
    
                    // There are a few timesteps where the two acceleration
                    // estimates drastically diverge. This appears to be around
                    // stage separation events. For now, simply check for errors
                    // and fallback to euler integration.
                    let acc_mag_ratio = magnitude(acceleration.x, acceleration.y) / magnitude(acc_rk4.x, acc_rk4.y);
                    if (acc_mag_ratio - 1.0).abs() < 0.02 { // 2% tolerance
                        position.x = pos_rk4.x;
                        position.y = pos_rk4.y;
                        velocity.x = vel_rk4.x;
                        velocity.y = vel_rk4.y;
                        acceleration = acc_rk4;
                    } else {
                        //println!("{:?} {:?} {:?} {:?}", i + 1, timestep, (time * 10000.0).round() / 10000.0, (acc_mag_ratio * 1000000.0).round() / 1000000.0);
                    }
                }

                // NOW mutably update prop after half-timestep
                mass_prop_remaining[i] -= prop_used;
    
                // Before stage separation i, stages >= i move together, so
                // overwrite the telemetries for stages >= i identically here.
                let telem_indices = if time < t2 {(i..stages.len()).collect()} else {vec![i]};
                for idx in telem_indices {
                    let radius = magnitude(position.x, position.y);
                    let altitude = radius - radius_earth;
                    if !update_telemetry[idx] {
                        // pass
                    } else if altitude < 0.0 {
                        // We either landed or crashed into the dirt.
                        telemetries[idx] = telemetries[idx][0..timestep].to_vec();
                        update_telemetry[idx] = false;
                    } else {
                        let telem = &mut telemetries[idx];
                        // Integrate trajectory / update telemetry
                        telem[timestep].acceleration.x = acceleration.x;
                        telem[timestep].acceleration.y = acceleration.y;

                        telem[timestep+1].velocity.x = velocity.x;
                        telem[timestep+1].velocity.y = velocity.y;

                        telem[timestep+1].position.x = position.x;
                        telem[timestep+1].position.y = position.y;

                        let density = barometric_density(altitude);
                        telem[timestep+1].barometric_density = density;
                        // convert to pressure in units of bar
                        let force_tot_bar = (mrockets::cross_sectional_area * standard_pressure);
                        telem[timestep+1].dynamic_pressure = force_drag_mag / force_tot_bar;
                    }
                }
            }
        }

        let elapsed_time = now.elapsed();
        println!("simulation time {} milliseconds.", elapsed_time.as_millis());
        telemetries.to_vec()
    }

    pub fn my_last(lst: Vec<f64>) -> f64 {
        return lst[lst.len()-1];
    }

    pub fn score_telemetries(orbit: bool, telemetries: & Vec<Vec<Telemetry>>, times: &Vec<f64>) -> f64 {
        // Only care about final stage for orbit check and delta_velocity
        let telemetry = &telemetries[telemetries.len()-1];

        let downranges: Vec<f64> = telemetries.iter().map(|t| my_last(get_downranges(t, times))).collect();

        // NOTE: The various score components will in general have different units.
        // This is okay!
        // NOTE: Use absolute value here instead of squaring the residual; otherwise,
        // the return-to-launch-site penalty can outweigh the dV to orbit!
        // Only care about initial stages for rtls penalty
        let rtls_penalty: f64 = downranges[0..downranges.len()-1].iter().map(|d| d.abs()).sum();

        // Check if we failed to reach orbit
        if telemetry.len() < num_timesteps {
            // If we have not reached orbit yet,
            // we do NOT want discontinuities in the objective function, because
            // that will cause an 'activity cliff' and prevent the algorithm from
            // improving failing solutions. Instead, return something that will
            // monotonically increase as we get closer to reaching orbit, such as
            // the downrange distance of the last stage.
            if orbit {
                return -f64::INFINITY;
            } else {
                // Divide by 100 so this is numerically smaller than the dV
                // required to achieve orbit.
                return downranges[downranges.len()-1] / 100.0;
            }
        }
        let velocities: Vec<f64> = (0..telemetry.len()-1).map(|i| magnitude(telemetry[i].velocity.x, telemetry[i].velocity.y)).collect();
        //let delta_velocity = velocities.iter().max();
        let delta_velocity = velocities.into_iter().max_by(|a, b| a.partial_cmp(b).unwrap());

        let score = delta_velocity.unwrap() - rtls_penalty;
        return score;
    }
}