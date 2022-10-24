//use std::f64::consts::PI;

pub mod mconstants {
    pub const mass_earth: f64 = 5.972e+24; // kg
    pub const radius_earth: f64 = 6378000.0; // m
    pub const big_G: f64 = 6.6743e-11; // m^3 kg^-1 s^-2
                                       // little_g in utils.rs
    pub const ideal_gas_constant: f64 = 8.3144598; // N·m/(mol·K)
    pub const molar_mass_air: f64 = 0.0289644; // kg/mol
    pub const tangental_velocity_earth: f64 =
        (radius_earth * 2.0 * std::f64::consts::PI) / (60.0 * 60.0 * 24.0); // 463.8 m/s
    pub const standard_pressure: f64 = 101325.0; // Pascal = N / m^2

    // NOTE: The humble .sqrt() method is defined by cases (pos & neg), so
    // it cannot be used to define global constants; define the square instead.
    pub const velocity_escape_2: f64 = 2.0 * big_G * mass_earth / radius_earth;
}
