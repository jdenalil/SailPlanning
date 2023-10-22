use eqsolver::single_variable::FDNewton;
use pathfinding::prelude::astar;

// PosTime struct for nodes
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct PosTime(i32, i32, u32);

#[derive(Clone, Copy, Debug)]
struct Current {
    magnitude: f64,
    direction: f64,
}

// scaling factor used before converting floats to ints
static SCALING_FACTOR: f64 = 1000.0;
// number of speeds used to discretize speed range
static NUM_SPEEDS_TO_SEARCH: u32 = 10;

type EnergyFunction = fn(f64) -> f64;

// LIMITATION: since the a-star algo I'm using can only handle int values, I'm scaling the hueristic, speed, and time to ints so I can pass them around inside the planner

pub fn print_plan(goal_x: i32, goal_y: i32, goal_time: f64, current_magnitude: f64, current_direction_from_north: f64, max_boat_speed: f64, energy_use_fn: EnergyFunction) {
    let result = run_astar(
        energy_use_fn,
        PosTime(goal_x, goal_y, (goal_time * SCALING_FACTOR) as u32), 
        Current { magnitude: current_magnitude, direction: current_direction_from_north }, 
        max_boat_speed
    );
    match result {
        Some(t) => {
            for p in t.0 {
                println!("x {} y {} time {}", p.0, p.1, p.2 as f64 / SCALING_FACTOR);
            }
            println!("found power cost {}", t.1 as f64 / SCALING_FACTOR);
        }
        None => {}
    }
}

fn run_astar(energy_use_fn: EnergyFunction, goal: PosTime, current: Current, max_speed: f64) -> Option<(Vec<PosTime>, u32)> {
    // NOTE: Hueristic Approx. must be greater than the real cost for this to guarantee an optimal solution
    astar(
        &PosTime(0, 0, 0),
        |p: &PosTime| find_successors(energy_use_fn, p, &goal, &current, max_speed),
        |p: &PosTime| calc_power_hueristic(energy_use_fn, p, &goal, &current, max_speed),
        |p: &PosTime| p == &goal,
    )
}

fn find_successors(
    energy_use_fn: EnergyFunction,
    point: &PosTime,
    goal: &PosTime,
    current: &Current,
    max_speed: f64,
) -> Vec<(PosTime, u32)> {
    // if we're at the goal location but haven't hit the goal time, only allow a hold at location
    if point.0 == goal.0 && point.1 == goal.1 && point.2 < goal.2 {
        // calculate time remaining and energy needed to sail against current until the goal time
        return vec![(
            goal.clone(),
            calc_hold_power(energy_use_fn, goal.2, point.2, current.magnitude),
        )];
    }

    // generate all possible movements
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

    // Generate NUM_SPEEDS_TO_SEARCH possible speeds in the range [0.0, max_speed]
    let speeds: Vec<f64> = (0..=NUM_SPEEDS_TO_SEARCH)
        .map(|i| (i as f64) * max_speed / NUM_SPEEDS_TO_SEARCH as f64)
        .collect();

    // Now, for each point, let's try each speed, filtering unreachable point / speed combos
    points
        .into_iter()
        .flat_map(|next_point| {
            speeds.iter().filter_map(move |&speed| {
                calculate_traversal_time(point, &next_point, current, speed).map(|addl_time| {
                    (
                        PosTime(next_point.0, next_point.1, point.2 + addl_time), // next_point location and time
                        calc_traversal_power(energy_use_fn, addl_time, speed), // cost to get to next_point
                    )
                })
            })
        })
        .collect()
}

fn calc_power_hueristic(
    energy_use_fn: EnergyFunction,
    point: &PosTime,
    goal: &PosTime,
    current: &Current,
    boat_water_speed: f64,
) -> u32 {
    match calculate_traversal_time(point, goal, current, boat_water_speed) {
        Some(traversal_time) => {
            // return traversal power + hold power
            calc_traversal_power(energy_use_fn, traversal_time, boat_water_speed)
                + calc_hold_power(energy_use_fn, goal.2, point.2 + traversal_time, current.magnitude)
        }
        None => {
            // return a very high power value - don't search here!
            // we can't return u32::MAX bc we run into addition overflows
            u16::MAX as u32
        }
    }
}

fn calculate_traversal_time(
    point: &PosTime,
    goal: &PosTime,
    current: &Current,
    boat_water_speed: f64,
) -> Option<u32> {
    // first calculate the distance and direction to the goal point
    let x_distance: i32 = goal.0 - point.0;
    let y_distance: i32 = goal.1 - point.1;
    let goal_distance = ((x_distance.pow(2) + y_distance.pow(2)) as f64).sqrt();
    let goal_direction = (x_distance as f64).atan2(y_distance as f64);

    // set up equation for time to get to local goal point
    let time_equation = |time_elapsed: f64| {
        let x: f64 = (goal_distance * goal_direction.sin())
            - (time_elapsed * current.magnitude * current.direction.sin());
        let y: f64 = (goal_distance * goal_direction.cos())
            - (time_elapsed * current.magnitude * current.direction.cos());
        ((x.powi(2) + y.powi(2)).sqrt() / boat_water_speed) - time_elapsed
    };

    // calculate time to reach goal given dynamics and return based on calculation success / failure
    match FDNewton::new(time_equation).solve(0.5) {
        Ok(time_elapsed) => {
            let time_elapsed_scaled = (time_elapsed * SCALING_FACTOR) as u32;
            if time_elapsed_scaled + point.2 > goal.2 {
                // traversal takes us beyond goal time
                None
            } else {
                // calc scaled power and time
                Some(time_elapsed_scaled)
            }
        }
        Err(_error) => {
            // Calculation not possible
            None
        }
    }
}

fn calc_hold_power(
    energy_use_fn: EnergyFunction,
    goal_time: u32,
    point_time: u32,
    current_speed: f64,
) -> u32 {
    ((goal_time - point_time) as f64 * energy_use_fn(current_speed)) as u32
}

fn calc_traversal_power(energy_use_fn: EnergyFunction, traversal_time: u32, traversal_speed: f64) -> u32 {
    (traversal_time as f64 * energy_use_fn(traversal_speed)) as u32
}
