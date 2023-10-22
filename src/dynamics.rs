#[allow(dead_code)]
pub fn exp_energy_use(boat_water_speed: f64) -> f64 {
    boat_water_speed * boat_water_speed
}

#[allow(dead_code)]
pub fn linear_energy_use(boat_water_speed: f64) -> f64 {
    boat_water_speed * 1.2
}

#[allow(dead_code)]
pub fn piecewise_energy_use(boat_water_speed: f64) -> f64 {
    if boat_water_speed == 0.0 {
        // we're just floating
        0.0
    } else if boat_water_speed <= 2.5 {
        boat_water_speed * 1.1 + 1.0
    } else if boat_water_speed <= 7.5 {
        boat_water_speed * 1.5 + 2.0
    } else {
        boat_water_speed * 1.25 + 3.0
    }
}
