
use pathfinding::prelude::astar;
use eqsolver::single_variable::FDNewton;
use crate::dynamics::energy_use;

// LIMITATION: since the astar algo I'm using can only handle int values, I'm scaling the hueristic, speed, and time to ints so I can pass them around inside the planner

// scaling factor used before converting floats to ints
pub static SCALING_FACTOR: f64 = 1000.0;
// number of speeds used to decritize speed range
static NUM_SPEEDS_TO_SEARCH: u32 = 10;


// PosTimeSpeed struct for nodes
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PosTimeSpeed(pub i32, pub i32, pub u32, pub u32);

pub struct UnscaledPosTimeSpeed {
    pub x: i32, 
    pub y: i32, 
    pub time: f64, 
    pub speed: f64,
}

pub fn run_astar(goal: PosTimeSpeed, current_speed: f64, current_direction: f64, max_speed: f64) -> Option<(Vec<UnscaledPosTimeSpeed>, f64)> {
    // NOTE: Hueristic Approx. MUST be greater than the real cost for this to be optimal
    let result: Option<(Vec<PosTimeSpeed>, u32)> = astar(
        &PosTimeSpeed(0, 0, 0, 0),
        |p: &PosTimeSpeed| p.successors(&goal, current_speed, current_direction, max_speed),
        |p: &PosTimeSpeed| calc_power(p, &goal, current_speed, current_direction, max_speed), // Hueristic is energy use at max speed - this will almost always be a pessimistic estimiate
        |p: &PosTimeSpeed| goal_reached(p, &goal),
    );
    match result {
        Some((pos_time_speed_vec, power_use)) => {
            let processed_result: Vec<UnscaledPosTimeSpeed> = pos_time_speed_vec
                .into_iter()
                .map(|pos_time_speed| UnscaledPosTimeSpeed {
                    x: pos_time_speed.0,
                    y: pos_time_speed.1,
                    time: pos_time_speed.2 as f64 / SCALING_FACTOR,
                    speed: pos_time_speed.3 as f64 / SCALING_FACTOR,
                })
                .collect();
            Some((processed_result, power_use as f64 / SCALING_FACTOR))
        }
        None => None,
    }
}


impl PosTimeSpeed {
    fn successors(&self, goal: &PosTimeSpeed, current_speed: f64, current_direction: f64, max_speed: f64) -> Vec<(PosTimeSpeed, u32)> {
        let &PosTimeSpeed(x, y, time, _) = self;

        // if we're at the end location but haven't hit the goal time, hold at location
        if x == goal.0 && y == goal.1 && time < goal.2 {
            // if so, calculate time remaining and energy needed to sail against current until the goal time
            // Return a single element with goal position, goal time, current speed, and power needed
            return vec![
                (PosTimeSpeed(goal.0, goal.1, goal.2, (current_speed * SCALING_FACTOR) as u32), 
                calc_hold_power(goal.2, time, 0, current_speed)),
            ];
        }

        // we aren't at the goal yet, let's first generate all possible movements
        let points: Vec<(i32, i32)> = vec![
            (x + 1, y), 
            (x - 1, y), 
            (x, y + 1), 
            (x, y - 1), 
            (x + 1, y + 1), 
            (x + 1, y - 1), 
            (x - 1, y + 1), 
            (x - 1, y - 1),
        ];

        // Generate 10 possible speeds in the range [0.0, max_speed] (inclusive of the bounds)
        let speeds: Vec<f64> = (0..=NUM_SPEEDS_TO_SEARCH).map(|i| (i as f64) * max_speed / NUM_SPEEDS_TO_SEARCH as f64).collect();

        // Now, for each point, let's try each speed, filtering for None returns meaning unreachable 
        let successors: Vec<(PosTimeSpeed, u32)> = points
            .into_iter()
            .flat_map(|point| {
                speeds.iter().filter_map(move |&speed| {
                    calculate_traversal_time_power(self, point, goal.2, current_speed, current_direction, speed).map(|(power_use, addl_time)| {
                        (PosTimeSpeed(point.0, point.1, time + addl_time, (speed * SCALING_FACTOR) as u32), power_use)
                    })
                })
            }).collect();
        
        for s in &successors {
            if s.0.0 == goal.0 && s.0.1 == goal.1 {
                println!("goal reached");
            }
        }

        successors
    }
}


fn calculate_traversal_time_power(point: &PosTimeSpeed, goal: (i32, i32), target_time: u32, current_speed: f64, current_direction: f64, boat_water_speed: f64) -> Option<(u32, u32)> {
    // first calculate the distance and direction to the goal point
    let x_distance: i32 = goal.0 - point.0;
    let y_distance: i32 = goal.1 - point.1;
    if x_distance == 0 && y_distance == 0 {
        // we are at goal, don't need to do calculations
        return Some((0, 0))
    }
    let goal_distance = ((x_distance.pow(2) + y_distance.pow(2)) as f64).sqrt();
    let goal_direction = (x_distance as f64).atan2(y_distance as f64);

    // equations for time to get to local goal point
    let time_equation = |time_elapsed: f64| {
        let x: f64 = (goal_distance * goal_direction.sin()) - (time_elapsed * current_speed * current_direction.sin());
        let y: f64 = (goal_distance * goal_direction.cos()) - (time_elapsed * current_speed * current_direction.cos());
        ((x.powi(2) + y.powi(2)).sqrt() / boat_water_speed) - time_elapsed
    };

    // calculate time to reach goal given dynamics and return based on calculation success / failure
    match FDNewton::new(time_equation).solve(0.5) {
        Ok(time_elapsed) => {
            let time_elapsed_scaled = time_elapsed * SCALING_FACTOR;
            // The calculation was successful
            // filter out invalid point times
            if time_elapsed_scaled as u32 + point.2 > target_time {
                // takes too long to traverse, cut the node
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

// wrapper function for calc calculate_traversal_time_power that returns power or 'high power value'
// we need this becuase astar needs a hueristic estimate for all points
pub fn calc_power(point: &PosTimeSpeed, goal: &PosTimeSpeed, current_speed: f64, current_direction: f64, boat_water_speed: f64) -> u32 {
    match calculate_traversal_time_power(point, (goal.0, goal.1), goal.2, current_speed, current_direction, boat_water_speed) {
        Some(power_time) => {
            // if the time returned from the hueristic is greater than the arrival time, return very high value
            // otherwise just return the energy required
            if power_time.1 > goal.2 {
                (SCALING_FACTOR * SCALING_FACTOR) as u32
            } else {
                power_time.0 + calc_hold_power(goal.2, point.2, power_time.1, current_speed)
            }
        }
        None => {
            // return high energy value - don't search here!
            (SCALING_FACTOR * SCALING_FACTOR) as u32
        }
    }
}

fn calc_hold_power(goal_time: u32, point_time: u32, traversal_time: u32, current_speed: f64) -> u32 {
    ((goal_time - point_time - traversal_time) as f64 * energy_use(current_speed)) as u32
}

fn goal_reached(point: &PosTimeSpeed, goal: &PosTimeSpeed) -> bool {
    point.0 == goal.0 && point.1 == goal.1 && point.2 == goal.2
}


#[cfg(test)]
mod tests {
    use super::calc_power;

    #[test]
    fn test_add() {
        assert_eq!(my_module::add(2, 3), 5);
    }
}