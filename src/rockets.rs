pub mod mrockets {
    use std::f64::consts::PI;

    use crate::rkt_types::mrkt_types::{Control, Engine, Stage};
    use crate::utils::mutils::little_g;

    pub fn get_burn_time(mass_prop: f64, engines: & Vec<Engine>) -> f64 {
        let dmdt: f64 = engines.iter().map(|e| e.dmdt).sum();
        /*let mut dmdt: f64 = 0.0;
        for e in &engines {
            dmdt += e.dmdt;
        }*/
        mass_prop / dmdt
    }

    // CF6-50 Turbofan
    pub const CF6_mass: f64 = 3960.0; // kg
    pub const CF6_TWR: f64 = 6.01; // unitless
    pub const CF6_SFC: f64 = 0.0105; // (kg/s)/kN
    pub const CF6_thrust: f64 = (CF6_mass * little_g) * CF6_TWR; // Newton
    pub const CF6_fuel_rate: f64 = (CF6_thrust / 1000.0) * CF6_SFC;
    //println!("CF6_thrust (kN) {:?}", CF6_thrust);
    //println!("CF6_thrust (Tons) {:?}", CF6_thrust / (little_g * 1000.0));
    //println!("CF6_fuel_rate (kg/s) {:?}", CF6_fuel_rate);
    pub const turbofan_CF6: Engine = Engine {
        dmdt: CF6_fuel_rate,
        velocity_exhaust: 95000.0,
        diameter: 2.19,
        mass: CF6_mass,
    };

    pub fn num_engines_exterior(diameter_rocket: f64, diameter_engine: f64) -> u32 {
        // Close enough for a first approximation.
        (PI * (diameter_rocket - diameter_engine) / diameter_engine) as u32
    }

    pub fn triangular_number(n: u32) -> u32 {
        (n * (n + 1) / 2) as u32
    }

    pub fn circle_packing_number(n: u32) -> u32 {
        1 + 6 * triangular_number(n - 1)
    }

    //println!("circle_packing_numbers 1 to 13");
    //println!({:?}, (1..14).iter().map(|n| circle_packing_number(n)).collect());

    // Rocket / staging setup
    pub const cross_sectional_area: f64 = PI * 4.5 * 4.5; // m^2

    pub const r2: Engine = Engine {
        dmdt: 650.0,
        velocity_exhaust: 3200.0,
        diameter: 1.3,
        mass: 1600.0,
    };

    pub const r2vac: Engine = Engine {
        dmdt: 650.0,
        velocity_exhaust: 3560.0,
        diameter: 2.4,
        mass: 1600.0,
    };

    pub fn superheavy(ctrls: Vec<Control>) -> Stage {
        let engs = vec![r2; 33];
        let massprop = 3400000.0;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 200000.0,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: 0.85 * burntime, // Save for boostback burn
            controls: ctrls,
        }
    }

    fn starship_engines() -> Vec<Engine> {
        let engines = &mut vec![r2; 3];
        engines.append(&mut vec![r2vac; 6]);
        engines.to_vec()
    }

    pub fn starship(ctrls: Vec<Control>) -> Stage {
        let engs = starship_engines();
        let massprop = 1200000.0;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 100000.0,
            mass_payload: 120000.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: burntime,
            controls: ctrls,
        }
    }

    // i.e. 12 hexagonal rings of turbofans, with the central turbofan + inner 3 rings removed.
    // This should give about 10% more thrust than superheavy, and about 5X more dry mass. (1425 Tons...)
    // Note that this dry mass estimate does not include any of the structural mass
    // necessary to actually connect the turbofans together and to the rocket, but
    // it also doesn't include any mass savings you would get by designing an
    // optimized turbfon specifically for this application.
    pub fn num_fans() -> u32 {
        circle_packing_number(12) - circle_packing_number(4) // 360 = 397 - 37
    }

    pub fn turbostage(ctrls: Vec<Control>) -> Stage {
        let engs = vec![turbofan_CF6; num_fans as usize];
        let massprop = 50000.0;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: num_fans() as f64 * CF6_mass,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: burntime,
            controls: ctrls,
        }
    }

    pub fn superheavy_t(ctrls: Vec<Control>) -> Stage {
        let engs = vec![r2; 33];
        let massprop = superheavy(vec![]).mass_prop - turbostage(vec![]).mass_prop;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 200000.0,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: 0.85 * burntime, // Save for boostback burn
            controls: ctrls,
        }
    }

    // In thrust we trust!
    pub fn superheavy_a(ctrls: Vec<Control>) -> Stage {
        let engs = vec![r2; 33 + 25];
        let massprop = 2200000.0;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 200000.0,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: 0.91 * burntime, // Save for boostback burn
            controls: ctrls,
        }
    }

    pub fn superheavy_b(ctrls: Vec<Control>) -> Stage {
        let engs = vec![r2; 30];
        let massprop = 1200000.0;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 100000.0,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: 0.77 * burntime, // Save for boostback burn
            controls: ctrls,
        }
    }

    pub fn superheavy_a_t(ctrls: Vec<Control>) -> Stage {
        let engs = vec![r2; 33 + 25];
        let massprop = superheavy_a(vec![]).mass_prop - turbostage(vec![]).mass_prop;
        let burntime = get_burn_time(massprop, & engs);
        Stage {
            mass_dry: 200000.0,
            mass_payload: 0.0,
            mass_prop: massprop,
            engines: engs,
            burn_time: 0.91 * burntime, // Save for boostback burn
            controls: ctrls,
        }
    }
}
