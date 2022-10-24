pub mod mcontrols {
    use std::f64::consts::PI;
    use rand::prelude::*;
    use crate::rkt_types::mrkt_types::{Control};

    // Controls (piecewise constant, all times are in seconds)
    pub const uptime: f64 = 10.0;
    pub const numturns1: i32 = 10;
    pub fn turntime1(bt: f64) -> f64 {
        (bt - uptime) / numturns1 as f64
    }
    pub fn turntime1a(bt: f64) -> f64 {
        turntime1(bt)
    }
    pub fn turntime1b(bt: f64) -> f64 {
        bt / numturns1 as f64
    }
    pub const numturns2: i32 = 10;
    pub const turntime2: f64 = 5.0;

    // Simplest possible control: straight up, then straight sideways.
    pub const up: Control = Control{t1: 0.0, t2: uptime, force_phi: 0.0, force_mag: 1.0};
    pub const over: Control = Control{t1: uptime, t2: 1000.0, force_phi: PI / 2.0, force_mag: 1.0};
    pub const controls_up_over: [Control; 2] = [up, over];

    pub fn piecewise_turn(init_time: f64, init_angle: f64, final_angle: f64, numturns: i32, turntime: f64) -> Vec<Control> {
        (0..numturns).into_iter().map(|r| r as f64).map(|r| Control {
            t1: init_time + r*turntime,
            t2: init_time + (r+1.0)*turntime,
            force_phi: init_angle + (final_angle - init_angle)*(r+1.0)/numturns as f64,
            force_mag: 1.0,
        }).collect()
    }

    pub fn gravity_turn1(bt: f64) -> Vec<Control> {
        piecewise_turn(0.0, 0.0, PI/4.0, numturns1, turntime1(bt))
    }
    pub fn gravity_turn1a(bt: f64) -> Vec<Control> {
        piecewise_turn(0.0, 0.0, PI/6.0, numturns1, turntime1a(bt))
    }
    pub fn gravity_turn1b(bt: f64) -> Vec<Control> {
        piecewise_turn(0.0, PI/6.0, PI/4.0, numturns1, turntime1b(bt))
    }

    pub const staging_time: f64 = 3.0;
    pub const staging_delay: Control = Control{t1: 0.0, t2: staging_time, force_phi: 0.0, force_mag: 0.0};

    pub fn gravity_turn2() -> Vec<Control> {
        piecewise_turn(0.0, PI/4.0, PI/2.0, numturns2, turntime2)
    }
    pub const over12: Control = Control{t1: 0.0, t2: 1000.0, force_phi: PI / 2.0, force_mag: 1.0};
    pub const boostback: Control = Control{t1: 0.0, t2: 1000.0, force_phi: - PI / 2.0, force_mag: 0.4}; // minus = backward, and using 40% thrust

    pub fn reduce_times(controls: & Vec<Control>) -> Vec<Control> {
        let mut time: f64 = 0.0;
        //let new_controls = &mut vec![];
        //for c in controls {
        let mkctrl = |x:(usize, &Control)| {
            let (i, c) = x;
            let delta_t = c.t2 - c.t1;
            let control = Control{t1: time, t2: time + delta_t, force_phi: c.force_phi, force_mag: c.force_mag};
            //new_controls.push(control);
            time += delta_t;
            control
        };
        controls.iter().enumerate().map(mkctrl).collect()
        //*new_controls
    }

    pub fn perturb_controls_phi_random(controls: & Vec<Control>, scale: f64, index_min: i32, index_max: i32) -> Vec<Control> {
        //let new_controls = &mut vec![];
        //for (i, c) in controls.iter().enumerate() {
        let mkctrl = |x:(usize, &Control)| {
            let (i, c) = x;
            // Skip controls out of bounds
            if ! (index_min as usize <= i && i < index_max as usize) {
                let new_control = Control{t1: c.t1, t2: c.t2, force_phi: c.force_phi, force_mag: c.force_mag};
                //new_controls.push(new_control);
                new_control
            } else {
                let mut rng = rand::thread_rng();
                let y: f64 = rng.gen(); // generates a float between 0 and 1
                let delta_phi = (y * 2.0 - 1.0) * (PI / 2.0) / scale;
                let new_control = Control{t1: c.t1, t2: c.t2, force_phi: c.force_phi + delta_phi, force_mag: c.force_mag};
                //new_controls.push(new_control);
                new_control
            }
        };
        controls.iter().enumerate().map(mkctrl).collect()
        //*new_controls
    }
}