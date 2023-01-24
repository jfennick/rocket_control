pub mod mrkt_types {
    use pyo3::pyclass;

    #[derive(Debug, Clone)]
    pub struct Engine {
        pub dmdt: f64,             // kg/s
        pub velocity_exhaust: f64, // m/s, in rocket frame
        pub diameter: f64,         // m
        pub mass: f64,             // kg
    }

    #[pyclass]
    #[derive(Debug, Clone)]
    pub struct Coords {
        #[pyo3(get)]
        pub x: f64,
        #[pyo3(get)]
        pub y: f64,
    }

    #[pyclass]
    #[derive(Debug, Clone)]
    pub struct Telemetry {
        #[pyo3(get)]
        pub position: Coords,
        #[pyo3(get)]
        pub velocity: Coords,
        #[pyo3(get)]
        pub acceleration: Coords,
        #[pyo3(get)]
        pub barometric_density: f64,
        #[pyo3(get)]
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

    #[pyclass]
    #[derive(Debug, Clone)]
    pub struct Control {
        // t1 < t2
        #[pyo3(get)]
        pub t1: f64,
        #[pyo3(get)]
        pub t2: f64,
        // In polar coordinates, this is the angle measured from vertical / radial
        #[pyo3(get)]
        pub force_phi: f64,
        #[pyo3(get)]
        pub force_mag: f64, // = 1.0
    }

    #[derive(Debug, Clone)]
    pub struct Stage<'a> {
        pub mass_dry: f64,     // kg
        pub mass_payload: f64, // kg
        pub mass_prop: f64,    // kg
        pub engines: Vec<Engine>,
        pub burn_time: f64,         // sec
        pub controls: &'a Vec<Control>, // = []
    }
}
