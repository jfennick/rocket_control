pub mod mrkt_types {

    #[derive(Debug, Clone)]
    pub struct Engine {
        pub dmdt: f64,             // kg/s
        pub velocity_exhaust: f64, // m/s, in rocket frame
        pub diameter: f64,         // m
        pub mass: f64,             // kg
    }

    #[derive(Debug, Clone)]
    pub struct Coords {
        pub x: f64,
        pub y: f64,
    }

    #[derive(Debug, Clone)]
    pub struct Telemetry {
        pub position: Coords,
        pub velocity: Coords,
        pub acceleration: Coords,
        pub barometric_density: f64,
        pub dynamic_pressure: f64,
    }

    #[derive(Debug, Clone)]
    pub struct TelemetrySOA {
        pub mass_prop_remaining: f64,
        pub positions: Vec<Coords>,
        pub velocities: Vec<Coords>,
        pub accelerations: Vec<Coords>,
        pub barometric_densities: Vec<f64>,
        pub dynamic_pressures: Vec<f64>,
        pub update: bool,
    }

    #[derive(Debug, Clone)]
    pub struct Control {
        // t1 < t2
        pub t1: f64,
        pub t2: f64,
        // In polar coordinates, this is the angle measured from vertical / radial
        pub force_phi: f64,
        pub force_mag: f64, // = 1.0
    }

    #[derive(Debug, Clone)]
    pub struct Stage {
        pub mass_dry: f64,     // kg
        pub mass_payload: f64, // kg
        pub mass_prop: f64,    // kg
        pub engines: Vec<Engine>,
        pub burn_time: f64,         // sec
        pub controls: Vec<Control>, // = []
    }
}
