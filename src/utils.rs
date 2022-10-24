pub mod mutils {

    use crate::constants::mconstants::{
        big_G, ideal_gas_constant, mass_earth, molar_mass_air, radius_earth,
    };

    pub fn magnitude(x: f64, y: f64) -> f64 {
        (x * x + y * y).sqrt()
    }

    pub fn cart2pol(x: f64, y: f64) -> (f64, f64) {
        let rho = magnitude(x, y);
        let phi = y.atan2(x);
        (rho, phi)
    }

    pub fn pol2cart(rho: f64, phi: f64) -> (f64, f64) {
        let x = rho * phi.cos();
        let y = rho * phi.sin();
        (x, y)
    }

    pub fn force_gravity(mass: f64, altitude: f64) -> f64 {
        // G m M / r ^ 2
        let rad = radius_earth + altitude;
        big_G * mass * mass_earth / (rad * rad)
    }

    //static little_g: f64 = force_gravity(1.0, 0.0); // 9.8 m/s^2
    /*
    error[E0015]: cannot call non-const fn `force_gravity` in statics
      --> src/utils.rs:27:24
       |
    27 | static little_g: f64 = force_gravity(1.0, 0.0); // 9.8 m/s^2
       |                        ^^^^^^^^^^^^^^^^^^^^^^^
       |
       = note: calls in statics are limited to constant functions, tuple structs and tuple variants
    */
    // Thus, we are forced to pre-calculate little_g
    pub const little_g: f64 = 9.7984299667611338691131095401942729949951171875000000000000000000; // m/s^2

    pub fn force_drag(
        density: f64,
        speed: f64,
        drag_coefficient: f64,
        cross_sectional_area: f64,
    ) -> f64 {
        // 1/2 rho v^2 c A
        0.5 * density * (speed * speed) * drag_coefficient * cross_sectional_area
    }

    pub fn barometric_density(altitude: f64) -> f64 {
        // See https://en.wikipedia.org/wiki/Barometric_formula#Density_equations
        // altitude (m), density (kg/m^3), temperature (K), temperature lapse rate (K/m)
        let piecewise_data = vec![
            (0.00000, 1.22500, 288.15, -0.0065),
            (11000.0, 0.36391, 216.65, 0.0),
            (20000.0, 0.08803, 216.65, 0.001),
            (32000.0, 0.01322, 228.65, 0.0028),
            (47000.0, 0.00143, 270.65, 0.0),
            (51000.0, 0.00086, 270.65, -0.0028),
            (71000.0, 0.000064, 214.65, -0.002),
            (84852.0, 0.00000, 000.00, 0.0),
        ]; // only altitude used here
        let piecewise_tail = &piecewise_data[1..];
        let zipped = piecewise_data.iter().zip(piecewise_tail.iter());
        for (i, (d, d2)) in zipped.enumerate() {
            let altitude_b: f64 = d.0;
            let altitude_b2: f64 = d2.0;
            if altitude >= altitude_b && altitude < altitude_b2 {
                let rho_b: f64 = d.1;
                let temp_b: f64 = d.2;
                let lapse_b: f64 = d.3;
                if lapse_b == 0.0 {
                    // TODO: Check this!!!
                    let numerator = (-1.0) * little_g * molar_mass_air * (altitude - altitude_b);
                    let denominator = ideal_gas_constant * temp_b;
                    return rho_b * std::f64::consts::E.powf(numerator / denominator);
                } else {
                    let base: f64 = temp_b / (temp_b + (altitude - altitude_b) * lapse_b);
                    let exponent: f64 =
                        1.0 + (little_g * molar_mass_air) / (ideal_gas_constant * lapse_b);
                    return rho_b * base.powf(exponent);
                }
            }
        }
        // else, we are in space
        return 0.0;
    }
}
