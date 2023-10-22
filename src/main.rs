mod planning;
mod dynamics;

use crate::planning::print_plan;

fn main() {
    // we're given a relative goal point, time of arrival, ocean current, and max_speed
    print_plan(15, 20, 10.0, 0.0, -0.7, 6.0);
}
