mod dynamics;
mod planning;

use crate::dynamics::exp_energy_use;
use crate::planning::print_plan;

fn main() {
    // we're given a relative goal point, time of arrival, ocean current, and max_speed
    print_plan(80, 50, 150.0, 5.0, -1.5, 12.0, exp_energy_use);
}
