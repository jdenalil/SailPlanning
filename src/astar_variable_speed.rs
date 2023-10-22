
use pathfinding::prelude::astar;
use eqsolver::single_variable::FDNewton;
use crate::dynamics::energy_use;

// LIMITATION: since the astar algo I'm using can only handle int values, I'm scaling the hueristic, speed, and time to ints so I can pass them around inside the planner

// scaling factor used before converting floats to ints
pub static SCALING_FACTOR: f64 = 1000.0;
// number of speeds used to decritize speed range
static NUM_SPEEDS_TO_SEARCH: u32 = 10;


// PosTime struct for nodes
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PosTime(pub i32, pub i32, pub u32);

pub struct UnscaledPosTime {
    pub x: i32, 
    pub y: i32, 
    pub time: f64,
}

pub struct Current {
    pub magnitude: f64,
    pub direction: f64,
}

pub fn run_astar(goal: PosTime, current: Current, max_speed: f64) -> Option<(Vec<UnscaledPosTime>, f64)> {
    // NOTE: Hueristic Approx. MUST be greater than the real cost for this to be optimal
    let result: Option<(Vec<PosTime>, u32)> = astar(
        &PosTime(0, 0, 0),
        |p: &PosTime| find_successors(p, &goal, current, max_speed),
        |p: &PosTime| calc_power_hueristic(p, &goal, current, max_speed), // Hueristic is energy use at max speed - this will almost always be a pessimistic estimiate
        |p: &PosTime| goal_reached(p, &goal),
    );
    match result {
        Some((pos_time_vec, power_use)) => {
            let processed_result: Vec<UnscaledPosTime> = pos_time_vec
                .into_iter()
                .map(|pos_time| UnscaledPosTime {
                    x: pos_time.0,
                    y: pos_time.1,
                    time: pos_time.2 as f64 / SCALING_FACTOR,
                })
                .collect();
            Some((processed_result, power_use as f64 / SCALING_FACTOR))
        }
        None => None,
    }
}

fn find_successors(point: &PosTime, goal: &PosTime, current: Current, max_speed: f64) -> Vec<(PosTime, u32)> {
    // if we're at the end location but haven't hit the goal time, hold at location
    if point.0 == goal.0 && point.1 == goal.1 && point.2 < goal.2 {
        // if so, calculate time remaining and energy needed to sail against current until the goal time
        // Return a single element with goal position, goal time, current speed, and power needed
        return vec![
            (PosTime(goal.0, goal.1, goal.2), 
            calc_hold_power(goal.2, point.2, 0, current.magnitude)),
        ];
    }

    // we aren't at the goal yet, let's first generate all possible movements
    let points: Vec<PosTime> = vec![
        PosTime(point.0 + 1, point.1, goal.2), 
        PosTime(point.0 - 1, point.1, goal.2), 
        PosTime(point.0, point.1 + 1, goal.2), 
        PosTime(point.0, point.1 - 1, goal.2), 
        PosTime(point.0 + 1, point.1 + 1, goal.2), 
        PosTime(point.0 + 1, point.1 - 1, goal.2), 
        PosTime(point.0 - 1, point.1 + 1, goal.2), 
        PosTime(point.0 - 1, point.1 - 1, goal.2),
    ];

    // Generate N possible speeds in the range [0.0, max_speed]
    let speeds: Vec<f64> = (0..=NUM_SPEEDS_TO_SEARCH).map(|i| (i as f64) * max_speed / NUM_SPEEDS_TO_SEARCH as f64).collect();

    // Now, for each point, let's try each speed, filtering for None returns meaning unreachable 
    let successors: Vec<(PosTime, u32)> = points
        .into_iter()
        .flat_map(|next_point| {
            speeds.iter().filter_map(move |&speed| {
                calculate_traversal(point, &next_point, current, speed).map(|(power_use, addl_time)| {
                    (PosTime(next_point.0, next_point.1, point.2 + addl_time), power_use)
                })
            })
        }).collect();
    successors
}


// wrapper function for calc calculate_traversal_time_power that returns power or 'high power value'
// we need this becuase astar needs a hueristic estimate for all points
fn calc_power_hueristic(point: &PosTime, goal: &PosTime, current: Current, boat_water_speed: f64) -> u32 {
    match calculate_traversal(point, goal, current, boat_water_speed) {
        Some(power_time) => {
            // return traversal power + hold power
            power_time.0 + calc_hold_power(goal.2, point.2, power_time.1, current.magnitude)
        }
        None => {
            // return high energy value - don't search here!
            (SCALING_FACTOR * SCALING_FACTOR) as u32
        }
    }
}

// return power and time for a given goal and water speed
fn calculate_traversal(point: &PosTime, goal: &PosTime, current: Current, boat_water_speed: f64) -> Option<(u32, u32)> {
    // first calculate the distance and direction to the goal point
    let x_distance: i32 = goal.0 - point.0;
    let y_distance: i32 = goal.1 - point.1;
    let goal_distance = ((x_distance.pow(2) + y_distance.pow(2)) as f64).sqrt();
    let goal_direction = (x_distance as f64).atan2(y_distance as f64);

    // set up equation for time to get to local goal point
    let time_equation = |time_elapsed: f64| {
        let x: f64 = (goal_distance * goal_direction.sin()) - (time_elapsed * current.magnitude * current.direction.sin());
        let y: f64 = (goal_distance * goal_direction.cos()) - (time_elapsed * current.magnitude * current.direction.cos());
        ((x.powi(2) + y.powi(2)).sqrt() / boat_water_speed) - time_elapsed
    };

    // calculate time to reach goal given dynamics and return based on calculation success / failure
    match FDNewton::new(time_equation).solve(0.5) {
        Ok(time_elapsed) => {
            let time_elapsed_scaled = time_elapsed * SCALING_FACTOR;
            if time_elapsed_scaled as u32 + point.2 > goal.2 {
                // traversal takes us beyond goal time
                None
            } else {
                // calc scaled power and time
                Some(((time_elapsed_scaled * energy_use(boat_water_speed)) as u32, time_elapsed_scaled as u32))
            }
        }
        Err(_error) => {
            // Calculation not possible
            None
        }
    }
}


fn calc_hold_power(goal_time: u32, point_time: u32, traversal_time: u32, current_speed: f64) -> u32 {
    ((goal_time - point_time - traversal_time) as f64 * energy_use(current_speed)) as u32
}

fn goal_reached(point: &PosTime, goal: &PosTime) -> bool {
    point.0 == goal.0 && point.1 == goal.1 && point.2 == goal.2
}

/* 
#[cfg(test)]
mod tests {
    use super::calc_power;

    #[test]
    fn test_add() {
        assert_eq!();
    }
}
*/