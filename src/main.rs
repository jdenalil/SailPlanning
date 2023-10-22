mod planning;
mod dynamics;

use crate::planning::print_plan;
use crate::dynamics::exp_energy_use;

fn main() {
    // we're given a relative goal point, time of arrival, ocean current, and max_speed
    print_plan(4, 2, 10.0, 1.0, -0.7, 6.0, exp_energy_use);
}
