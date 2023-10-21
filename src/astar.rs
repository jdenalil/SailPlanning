use pathfinding::prelude::astar;
use eqsolver::single_variable::FDNewton;

static SCALING_FACTOR: f64 = 1000.0;
static NOT_POSSIBLE_CUTOFF: f64 = 1000.0;
static SEARCH_DIST_CUTOFF: i32 = 1000;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Pos(pub i32, pub i32);

pub fn run_astar(goal: Pos, current_speed: f64, current_direction: f64, desired_speed: f64) -> f64 {
    let result = astar(&Pos(0, 0), |p| p.successors(current_speed, current_direction, desired_speed), |p| p.traversal_time(&goal, current_speed, current_direction, desired_speed),
                   |p| *p == goal);
    // TODO - return result from this
    match result {
        Some(t) => {
            t.1 as f64 / SCALING_FACTOR
        }
        None => {
            -1.0
        }
    }
}


impl Pos {
    fn traversal_time(&self, other: &Pos, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> u32 {
        let time = calculate_time_given_water_speed(self, other, current_speed, current_direction, constant_water_speed);
        return (SCALING_FACTOR * time) as u32
    }

    fn successors(&self, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> Vec<(Pos, u32)> {
        let &Pos(x, y) = self;

        // if we are searching very far away, end the search
        // TODO: do we need this?
        if (x.abs() > SEARCH_DIST_CUTOFF) || (y.abs() > SEARCH_DIST_CUTOFF) {
            println!("search cut off at {}, {}", x, y);
            let ret: Vec<(Pos, u32)> = vec![];
            return ret
        }

        // traversal_time used as the distance hueristic
        let valid_points: Vec<(Pos, u32)> = vec![Pos(x + 1, y), Pos(x - 1, y), Pos(x, y + 1), Pos(x, y - 1), Pos(x + 1, y + 1), Pos(x + 1, y - 1), Pos(x - 1, y + 1), Pos(x - 1, y - 1)]
            .into_iter()
            .filter_map(|p: Pos| {
                let d: u32 = self.traversal_time(&p, current_speed, current_direction, constant_water_speed);
                // filter out invalid traversal times
                if d <= (SCALING_FACTOR * NOT_POSSIBLE_CUTOFF) as u32 {
                    Some((p, d))
                } else {
                    None
                }
            })
            .collect();
        valid_points
    }
}


fn calculate_time_given_water_speed(point: &Pos, goal: &Pos, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> f64 {
    // TODO: have this return an OPTION
    // first calculate the distance and direction to the goal point
    let x_distance: i32 = goal.0 - point.0;
    let y_distance: i32 = goal.1 - point.1;
    let goal_distance = ((x_distance.pow(2) + y_distance.pow(2)) as f64).sqrt();
    let goal_direction = (x_distance as f64).atan2(y_distance as f64);

    let time_equation = |time_elapsed: f64| {
        let x: f64 = (goal_distance * goal_direction.sin()) - (time_elapsed * current_speed * current_direction.sin());
        let y: f64 = (goal_distance * goal_direction.cos()) - (time_elapsed * current_speed * current_direction.cos());
        ((x.powi(2) + y.powi(2)).sqrt() / constant_water_speed) - time_elapsed
    };

    // calculate time to reach goal given dynamics and return based on calculation success / failure
    match FDNewton::new(time_equation).solve(0.5) {
        Ok(time_elapsed) => {
            // The calculation was successful
            time_elapsed
        }
        Err(_error) => {
            // Calculation not possible, return very high value so the point will be filtered or not chosen
            NOT_POSSIBLE_CUTOFF + 1.0
        }
    }
}

