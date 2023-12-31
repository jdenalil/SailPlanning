mod dynamics;
mod planning;

use crate::dynamics::exp_energy_use;
use crate::planning::print_plan;

fn main() {
    // user can set the following parameters
    let goal_x = 8;
    let goal_y = 5;
    let goal_time = 8.5;
    let current_magnitude = 1.0;
    let current_direction_from_north = -1.5; // rads from true north
    let max_boat_speed = 12.0;
    let energy_use_fn = exp_energy_use;

    print_plan(goal_x, goal_y, goal_time, current_magnitude, current_direction_from_north, max_boat_speed, energy_use_fn);
}
