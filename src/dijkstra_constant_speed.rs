use pathfinding::prelude::dijkstra;
use eqsolver::single_variable::FDNewton;

static SCALING_FACTOR: f64 = 1000.0;
static SEARCH_DIST_CUTOFF: i32 = 500;
static MAX_TRAVERSAL_TIME: f64 = 10.0;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Pos(pub i32, pub i32);


pub fn run_dijkstra(goal: Pos, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> Option<(Vec<Pos>, f64)> {
    let mut result = dijkstra(
        &Pos(0, 0),
        |p| p.successors(current_speed, current_direction, constant_water_speed),
        |p| *p == goal,
    );
    match result {
        Some(t) => {
            Some((t.0, t.1 as f64 / SCALING_FACTOR))
        }
        None => {
            None
        }
    }
}


impl Pos {
    fn successors(&self, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> Vec<(Pos, u32)> {
        // unpack additional values here
        let &Pos(x, y) = self;

        // if we are searching very far away, end the search
        if (x.abs() > SEARCH_DIST_CUTOFF) || (y.abs() > SEARCH_DIST_CUTOFF) {
            println!("search cut off at {}, {}", x, y);
            return vec![]
        }

        let points: Vec<Pos> = vec![Pos(x + 1, y), Pos(x - 1, y), Pos(x, y + 1), Pos(x, y - 1), Pos(x + 1, y + 1), Pos(x + 1, y - 1), Pos(x - 1, y + 1), Pos(x - 1, y - 1)];

        // traversal_time used as the distance hueristic
        points.into_iter()
            .filter_map(|p: Pos| {
                calculate_traversal_time(self, &p, current_speed, current_direction, constant_water_speed)
                .map(|d| (p, d))
            })
            .collect()
    }
}

fn calculate_traversal_time(point: &Pos, goal: &Pos, current_speed: f64, current_direction: f64, constant_water_speed: f64) -> Option<u32> {
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
            if time_elapsed > MAX_TRAVERSAL_TIME {
                // takes too long to traverse, cut the node
                None
            } else {
                // calc scaled int time
                Some((SCALING_FACTOR * time_elapsed) as u32)
            }
        }
        Err(_error) => {
            // Calculation not possible
            None
        }
    }
}



fn find_optimal_speed(goal: Pos, current_speed: f64, current_direction: f64, time_goal: f64, max_speed: f64) -> f64 {
    // find the water speed that will get us to the goal in time
  
    let mut low_speed = 0.0;
    let mut high_speed = max_speed;
    let mut mid_speed = high_speed;
    let mut solved_arrival_time: Option<f64> = None;
  
    while (time_goal - solved_arrival_time.unwrap_or(-1.0)).abs() > 0.25 && (high_speed - low_speed) > 0.0001  { 
      mid_speed = (low_speed + high_speed) / 2.0;
      // println!("{}, {}", solved_arrival_time, mid_speed);
      solved_arrival_time = match run_dijkstra(goal, current_speed, current_direction, mid_speed) {
        Some(t) => {
          for p in t.0 {
            println!("x {}, y {}", p.0, p.1);
          }
          println!("{}, {}", t.1, mid_speed);
          Some(t.1)
        }
        None => {
            None
        }
      };
      if solved_arrival_time.is_none() {
        // search failed bc it is too slow
        low_speed = mid_speed;
      } else if time_goal > solved_arrival_time.unwrap() {
        high_speed = mid_speed;
      } else {
        low_speed = mid_speed;
      }
    }
    mid_speed
  }
  