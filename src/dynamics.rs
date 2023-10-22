pub fn energy_use(boat_water_speed: f64) -> f64 {
    if boat_water_speed < 0.0 {
        // TODO: return error
        println!("negative water speed");
        1000.0
    } else if boat_water_speed <= 0.1 {
        // we're just floating
        0.0
    } else if boat_water_speed <= 2.5 {
        // linear, with penalty for running
        // power use: 1.05 to 2.25
        boat_water_speed * 1.1 + 1.0
    } else if boat_water_speed <= 7.5 {
        // middle speed, not efficient
        // power use: 3.5 to 8.5
        boat_water_speed * 1.5 + 1.0
    } else {
        // high speed
        // power use: 7.75 to ?
        boat_water_speed * 1.25 + 3.0
    }
  }
